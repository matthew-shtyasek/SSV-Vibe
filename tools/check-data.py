"""Checks that the data files of SSV agree with each other and with the code, without starting Orbiter.

 1. meshres_*.h (group indices of the meshes in the code) against the mesh files: number of groups, and the name of the group of every index.
    A wrong index means a switch, a light or a talkback that moves or lights another part, or nothing.
 2. the meshes of 1: the vertex and triangle counts agree with the file, the triangles use existing vertices, there are no NaN values,
    and the textures that the meshes name exist in Textures.
 3. the scenarios: the blocks BEGIN_ and END_ are balanced.

Usage: python tools/check-data.py          (exit code 1 if there are errors; warnings do not change it)
"""
import math
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC = os.path.join(ROOT, 'Orbitersdk', 'Space Shuttle Vessel')
MESHES = os.path.join(ROOT, 'Meshes')
TEXTURES = os.path.join(ROOT, 'Textures')
SCENARIOS = os.path.join(ROOT, 'Scenarios')

errors = []
warnings = []
stats = {'headers': 0, 'groups': 0, 'meshes': 0, 'scenarios': 0}


def index_files(folder, ext):
    found = {}
    for dp, dn, fn in os.walk(folder):
        for f in fn:
            if f.lower().endswith(ext):
                found.setdefault(f.lower(), os.path.join(dp, f))
    return found


def parse_mesh(path):
    """Returns (list of groups, list of texture names). Group: dict(label, nv, nt, vertices, triangles)."""
    with open(path, encoding='latin-1') as f:
        lines = f.read().split('\n')
    groups = []
    textures = []
    i = 0
    cur = None
    in_textures = 0
    while i < len(lines):
        l = lines[i].strip()
        if l.startswith('LABEL'):
            cur = {'label': l.split(None, 1)[1].strip() if len(l.split(None, 1)) > 1 else ''}
            groups.append(cur)
        elif l.startswith('GEOM') and cur is not None:
            parts = l.split()
            nv = int(parts[1])
            nt = int(parts[2])
            cur['nv'] = nv
            cur['nt'] = nt
            cur['v'] = lines[i + 1:i + 1 + nv]
            cur['t'] = lines[i + 1 + nv:i + 1 + nv + nt]
            i += nv + nt
        elif l.startswith('TEXTURES'):
            in_textures = int(l.split()[1])
        elif in_textures > 0 and l:
            textures.append(l)
            in_textures -= 1
        i += 1
    return groups, textures


def check_meshres(textures_index, mesh_index):
    checked = set()
    for dp, dn, fn in os.walk(SRC):
        for f in fn:
            if not (f.lower().startswith('meshres') and f.lower().endswith('.h')):
                continue
            path = os.path.join(dp, f)
            text = open(path, encoding='latin-1').read()
            m = re.search(r'Input file:\s*(\S+\.msh)', text)
            n = re.search(r'NUMGRP_\w+\s*=\s*(\d+)', text)
            if not m or not n:
                continue
            meshname = m.group(1).lower()
            expected = {}
            for g in re.finditer(r'GRP_(\w+?)\s*=\s*(\d+);\s*//(.*)', text):
                expected[int(g.group(2))] = g.group(3).strip()
            if meshname not in mesh_index:
                warnings.append('%s: the mesh %s was not found' % (f, meshname))
                continue
            meshpath = mesh_index[meshname]
            stats['headers'] += 1
            stats['groups'] += len(expected)
            groups, textures = parse_mesh(meshpath)
            numgrp = int(n.group(1))
            if len(groups) != numgrp:
                errors.append('%s: NUMGRP is %d but %s has %d groups' % (f, numgrp, os.path.basename(meshpath), len(groups)))
            for idx, label in expected.items():
                if idx >= len(groups):
                    errors.append('%s: the group %d (%s) does not exist in %s' % (f, idx, label, os.path.basename(meshpath)))
                elif groups[idx]['label'] != label:
                    errors.append('%s: the group %d is "%s" in the header and "%s" in %s' % (f, idx, label, groups[idx]['label'], os.path.basename(meshpath)))
            # the groups of the mesh that the header does not name are only informative
            if meshpath not in checked:
                checked.add(meshpath)
                stats['meshes'] += 1
                check_mesh(meshpath, groups, textures, textures_index)


def check_mesh(path, groups, textures, textures_index):
    name = os.path.basename(path)
    for g in groups:
        if 'nv' not in g:
            continue
        if len(g['v']) != g['nv'] or len(g['t']) != g['nt']:
            errors.append('%s: the group %s has a vertex or triangle count that does not agree with the file' % (name, g['label']))
            continue
        for line in g['v']:
            for tok in line.split()[:8]:
                try:
                    v = float(tok)
                except ValueError:
                    errors.append('%s: the group %s has a vertex that is not a number' % (name, g['label']))
                    break
                if math.isnan(v) or math.isinf(v):
                    errors.append('%s: the group %s has a NaN vertex' % (name, g['label']))
                    break
        for line in g['t']:
            try:
                idx = [int(x) for x in line.split()[:3]]
            except ValueError:
                errors.append('%s: the group %s has a bad triangle' % (name, g['label']))
                break
            if len(idx) != 3 or min(idx) < 0 or max(idx) >= g['nv']:
                errors.append('%s: the group %s has a triangle with a vertex that does not exist' % (name, g['label']))
                break
    for t in textures:
        base = os.path.basename(t.replace('\\', '/')).lower()
        if base not in textures_index:
            warnings.append('%s: the texture %s was not found in Textures' % (name, t))


def check_scenarios():
    for dp, dn, fn in os.walk(SCENARIOS):
        for f in fn:
            if not f.lower().endswith('.scn'):
                continue
            path = os.path.join(dp, f)
            stats['scenarios'] += 1
            stack = []
            for n, line in enumerate(open(path, encoding='latin-1').read().split('\n'), 1):
                l = line.strip()
                if l.startswith('BEGIN_'):
                    stack.append(l[6:].split()[0] if l[6:].split() else '')
                elif l.startswith('END_'):
                    name = l[4:].split()[0] if l[4:].split() else ''
                    if not stack or stack[-1] != name:
                        errors.append('%s:%d: END_%s without BEGIN_%s' % (os.path.relpath(path, ROOT), n, name, name))
                        stack = []
                        break
                    stack.pop()
            if stack:
                errors.append('%s: BEGIN_%s is not closed' % (os.path.relpath(path, ROOT), stack[-1]))


def main():
    textures_index = index_files(TEXTURES, '.dds')
    textures_index.update(index_files(TEXTURES, '.bmp'))
    textures_index.update(index_files(TEXTURES, '.tga'))
    mesh_index = index_files(MESHES, '.msh')
    check_meshres(textures_index, mesh_index)
    check_scenarios()
    for w in warnings:
        print('warning: ' + w)
    for e in errors:
        print('ERROR: ' + e)
    print('checked: %(headers)d headers of meshes (%(groups)d group names), %(meshes)d meshes, %(scenarios)d scenarios' % stats)
    print('%d error(s), %d warning(s)' % (len(errors), len(warnings)))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
