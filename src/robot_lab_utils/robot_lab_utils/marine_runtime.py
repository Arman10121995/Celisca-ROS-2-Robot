"""Positive startup contract for the native marine force convention."""
import re
import subprocess


def require_fortress_mass_convention():
    try:
        result=subprocess.run(['ign','gazebo','--versions'],capture_output=True,text=True,timeout=5,check=True)
    except (OSError,subprocess.SubprocessError) as exc:
        raise RuntimeError('Native marine physics requires installed Gazebo Fortress 6.18 or newer in major version 6') from exc
    versions=[tuple(map(int,match)) for match in re.findall(r'\b(6)\.(\d+)\.(\d+)\b',result.stdout)]
    if not versions or max(versions)<(6,18,0):
        raise RuntimeError('Marine added-mass adapter targets Fortress 6.18+; found '+result.stdout.strip()+'. Review the native -Ma*acceleration convention before porting.')
    return '.'.join(map(str,max(versions)))
