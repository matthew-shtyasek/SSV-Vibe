/****************************************************************************
  This file is part of Space Shuttle Vessel

  Files of the flight data log (data of the DAP and of the vehicle for the study of the attitude hold)


  Space Shuttle Vessel is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License,
  or (at your option) any later version.

  Space Shuttle Vessel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Vessel; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA,
  or see http://www.gnu.org/licenses/gpl.html

  See http://spacenews.com/ for more details.

  **************************************************************************/
#ifndef _FLIGHTLOG_H_
#define _FLIGHTLOG_H_


#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <direct.h>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>


namespace flightlog
{
	/** Folder (in the Orbiter directory) of the files of the log. */
	inline constexpr const char* FOLDER = "SSV_FlightLogs";

	/**
	 * Name of a file of the log: FOLDER\<tag>_<yyyymmdd_hhmmss>.log (the characters of the tag that a file name can not have are replaced by '_').
	 * @return the length of the name
	 */
	inline int FileName( char* buf, size_t size, const char* tag, const tm& t )
	{
		char clean[48];
		size_t n = 0;
		for (; (tag != NULL) && (tag[n] != '\0') && (n < (sizeof( clean ) - 1)); n++)
		{
			const char c = tag[n];
			const bool ok = ((c >= '0') && (c <= '9')) || ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z')) || (c == '-') || (c == '_');
			clean[n] = ok ? c : '_';
		}
		clean[n] = '\0';
		return sprintf_s( buf, size, "%s\\%s_%04d%02d%02d_%02d%02d%02d.log", FOLDER, clean, t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec );
	}

	/**
	 * Text file of the log: opened at the first use. The lines wait in memory and a thread writes them once a second, so the simulation never waits
	 * for the disk (a crash loses at most the last second).
	 */
	class Writer
	{
		FILE* fp = NULL;
		bool failed = false;
		std::mutex mtx;
		std::condition_variable cv;
		std::string pending;
		bool stopping = false;
		std::thread worker;

		void Run( void )
		{
			std::string local;
			for (;;)
			{
				{
					std::unique_lock<std::mutex> lock( mtx );
					cv.wait_for( lock, std::chrono::seconds( 1 ), [this] { return stopping; } );
					local.swap( pending );
					if (local.empty() && stopping) return;
				}
				if (!local.empty())
				{
					fwrite( local.data(), 1, local.size(), fp );
					fflush( fp );
					local.clear();
				}
			}
		}
	public:
		Writer( void ) = default;
		Writer( const Writer& ) = delete;
		Writer& operator=( const Writer& ) = delete;
		~Writer( void ) { Close(); }

		bool IsOpen( void ) const { return fp != NULL; }

		/** Creates the folder and the file; @return false if that is not possible (it is not tried again until Close). */
		bool Open( const char* tag )
		{
			if (fp != NULL) return true;
			if (failed) return false;
			_mkdir( FOLDER );// it exists already, most of the time
			const time_t now = time( NULL );
			tm t;
			localtime_s( &t, &now );
			char name[160];
			FileName( name, sizeof( name ), tag, t );
			if (fopen_s( &fp, name, "w" ) != 0)
			{
				fp = NULL;
				failed = true;
				return false;
			}
			stopping = false;
			worker = std::thread( [this] { Run(); } );
			return true;
		}

		void Close( void )
		{
			if (fp != NULL)
			{
				{
					std::lock_guard<std::mutex> lock( mtx );
					stopping = true;
				}
				cv.notify_all();
				if (worker.joinable()) worker.join();
				fclose( fp );
			}
			fp = NULL;
			failed = false;
		}

		void Line( const char* format, ... )
		{
			if (fp == NULL) return;
			char buf[3072];
			va_list args;
			va_start( args, format );
			_vsnprintf_s( buf, sizeof( buf ), _TRUNCATE, format, args );
			va_end( args );
			std::lock_guard<std::mutex> lock( mtx );
			pending += buf;
			pending += '\n';
		}
	};
}


#endif// _FLIGHTLOG_H_
