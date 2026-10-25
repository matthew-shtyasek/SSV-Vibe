"""Reports the encoding of the C++ and C# sources of SSV, and converts the Windows-1252 files to UTF-8.

Most of the files are ASCII. Some have non-ASCII characters (mostly the degree sign) in Windows-1252 and others in UTF-8, which
breaks tools that expect UTF-8. A file is converted only if all its non-ASCII characters are in comments: a character in a string
or in the code would change the bytes sent to the displays (the MDU font uses the Windows-1252 codes), so those files are listed
and left alone.

Usage:
    python tools/check-encoding.py            report
    python tools/check-encoding.py --fix      convert the safe Windows-1252 files to UTF-8 (without BOM; the line endings are kept)
"""
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Orbitersdk', 'Space Shuttle Vessel')
EXTS = ('.cpp', '.h', '.hpp', '.c', '.cs', '.xaml')


def in_comment_only(text):
    """True if every non-ASCII character is in a // comment (a line comment before it on the same line) or in a /* */ block."""
    in_block = False
    for line in text.split('\n'):
        i = 0
        in_string = False
        while i < len(line):
            c = line[i]
            two = line[i:i + 2]
            if in_block:
                if two == '*/':
                    in_block = False
                    i += 2
                    continue
            elif in_string:
                if c == '\\':
                    i += 2
                    continue
                if c == '"':
                    in_string = False
                elif ord(c) > 127:
                    return False
            else:
                if two == '//':
                    break
                if two == '/*':
                    in_block = True
                    i += 2
                    continue
                if c == '"':
                    in_string = True
                elif ord(c) > 127:
                    return False
            i += 1
    return True


def main():
    fix = '--fix' in sys.argv
    cp1252 = []
    safe = []
    unsafe = []
    for dp, dn, fn in os.walk(ROOT):
        if os.sep + 'bin' in dp or os.sep + 'obj' in dp:
            continue
        for f in fn:
            if not f.lower().endswith(EXTS):
                continue
            p = os.path.join(dp, f)
            b = open(p, 'rb').read()
            try:
                b.decode('ascii')
                continue
            except UnicodeDecodeError:
                pass
            try:
                b.decode('utf-8')
                continue
            except UnicodeDecodeError:
                pass
            cp1252.append(p)
            text = b.decode('cp1252', errors='replace')
            (safe if in_comment_only(text) else unsafe).append((p, text, b))

    print('Windows-1252 files: %d (%d with the non-ASCII characters only in comments, %d with some in strings or code)' % (len(cp1252), len(safe), len(unsafe)))
    for p, _, _ in unsafe:
        print('  keeps Windows-1252: ' + os.path.relpath(p, ROOT))
    if fix:
        for p, text, b in safe:
            open(p, 'wb').write(text.encode('utf-8'))
            print('  converted: ' + os.path.relpath(p, ROOT))
    else:
        for p, _, _ in safe:
            print('  can be converted: ' + os.path.relpath(p, ROOT))
        print('Run with --fix to convert them.')


if __name__ == '__main__':
    main()
