/****************************************************************************
  This file is part of Space Shuttle Vessel

  Basic panel definitions


  Space Shuttle Vessel is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Space Shuttle Vessel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Vessel; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html or
  file SSV-LICENSE.txt for more details.

  **************************************************************************/
/******* SSV File Modification Notice *******
Date         Developer
2020/06/20   GLS
2021/01/20   GLS
2021/08/24   GLS
2021/12/26   GLS
2022/08/05   GLS
2022/09/29   GLS
********************************************/
/****************************************************************************
  This file is part of Space Shuttle Ultra

  Basic panel definitions



  Space Shuttle Ultra is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Space Shuttle Ultra is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Ultra; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html or
  file Doc\Space Shuttle Ultra\GPL.txt for more details.

  **************************************************************************/
#ifndef _ATLANTIS_PANEL_H_
#define _ATLANTIS_PANEL_H_

#include <OrbiterAPI.h>
#include <string>
#include <BasicPanel.h>
#include "../Atlantis.h"
#include "../CabinLightingData.h"
#include <cstring>
#include <cctype>
#include <cmath>


namespace vc
{

using namespace std;


class AtlantisPanel : public BasicPanel<Atlantis>
{
public:
	AtlantisPanel(Atlantis* _sts, const std::string& _name) : BasicPanel(_sts, _name), lightingChannel(cabinlight::PNL_NONE), dimMask(0), glow(1.0)
	{
		for (int i = 0; i < 32; i++) baseEmissive[i][0] = baseEmissive[i][1] = baseEmissive[i][2] = -1.0f;
	};
	virtual ~AtlantisPanel() {};
	Atlantis* STS() const { return V(); };

	void AddMeshes( const VECTOR3 &ofs ) override
	{
		if (mesh_idx == (UINT)-1)
		{
			VECTOR3 _ofs = ofs + panelofs;
			mesh_idx = STS()->AddMesh( hMesh, &_ofs );
			STS()->SetMeshVisibilityMode( mesh_idx, MESHVIS_VC );
		}
		return;
	}

	/** @return the channel of the panel lighting that dims this panel (cabinlight::PanelChannel) */
	int GetLightingChannel() const { return lightingChannel; }

	/**
	 * Sets the brightness of the panel lighting: the self illumination of the panel surfaces is scaled (1 = as it is in the mesh).
	 * @param scale [0, 1]
	 */
	void SetGlow( double scale )
	{
		if (fabs( scale - glow ) < 0.002) return;
		glow = scale;
		ApplyGlow();
	}

	void VisualCreated() override
	{
		BasicPanel::VisualCreated();
		ApplyGlow();
	}

	void SetMeshVisibility( bool visible )override
	{
		if (mesh_idx != (UINT)-1)
		{
			if (visible) STS()->SetMeshVisibilityMode( mesh_idx, MESHVIS_VC );
			else STS()->SetMeshVisibilityMode( mesh_idx, MESHVIS_NEVER );
		}
		return;
	}

protected:
	void DefineMesh( const char* meshname ) override
	{
		const VECTOR3 panelofs = _V( 0.0, 0.0, 0.0 );
		DefineMesh( meshname, panelofs );
		return;
	}

	void DefineMesh( const char* meshname, const VECTOR3& panelofs ) override
	{
		hMesh = oapiLoadMeshGlobal( meshname );
		mesh_idx = (UINT)-1;
		FindLighting( meshname );
		this->panelofs = panelofs;
		bHasOwnVCMesh = true;
		return;
	}

private:
	int lightingChannel;
	unsigned long dimMask;// materials of the mesh that the panel lighting dims
	double glow;
	float baseEmissive[32][3];// emissive color of the materials in the mesh (-1 = not read yet)

	/** Looks in the table of the cabin lighting for the mesh. */
	void FindLighting( const char* meshname )
	{
		const char* base = meshname;
		for (const char* c = meshname; *c != 0; c++)
		{
			if ((*c == '\\') || (*c == '/')) base = c + 1;
		}
		char lower[64];
		size_t n = 0;
		for (; (base[n] != 0) && (n < (sizeof( lower ) - 1)); n++) lower[n] = static_cast<char>(tolower( static_cast<unsigned char>(base[n]) ));
		lower[n] = 0;
		lightingChannel = cabinlight::PNL_NONE;
		dimMask = 0;
		for (int i = 0; i < cabinlight::PANEL_LIGHTING_COUNT; i++)
		{
			if (strcmp( lower, cabinlight::PANEL_LIGHTING[i].mesh ) == 0)
			{
				lightingChannel = cabinlight::PANEL_LIGHTING[i].channel;
				dimMask = cabinlight::PANEL_LIGHTING[i].dimMask;
				break;
			}
		}
	}

	void ApplyGlow()
	{
		if ((mesh_idx == (UINT)-1) || (dimMask == 0) || (STS()->Get_vis() == 0)) return;
		DEVMESHHANDLE hDev = STS()->GetDevMesh( STS()->Get_vis(), mesh_idx );
		if (hDev == NULL) return;
		for (unsigned int i = 0; i < 32; i++)
		{
			if ((dimMask & (1ul << i)) == 0) continue;
			MATERIAL mat;
			if (oapiMeshMaterial( hDev, i, &mat ) != 0) continue;
			if (baseEmissive[i][0] < 0.0f)
			{
				baseEmissive[i][0] = mat.emissive.r;
				baseEmissive[i][1] = mat.emissive.g;
				baseEmissive[i][2] = mat.emissive.b;
			}
			mat.emissive.r = baseEmissive[i][0] * static_cast<float>(glow);
			mat.emissive.g = baseEmissive[i][1] * static_cast<float>(glow);
			mat.emissive.b = baseEmissive[i][2] * static_cast<float>(glow);
			oapiSetMaterial( hDev, i, &mat );
		}
	}
};

}

#endif// _ATLANTIS_PANEL_H_
