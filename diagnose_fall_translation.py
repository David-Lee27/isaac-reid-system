r"""
diagnose_fall_translation.py

Checks the fall clip's root-joint translation range across all samples -
confirms/denies whether it carries large horizontal drift (which, left
unstripped, could shift a fallen character meters away from where they
collapsed - possibly explaining "the fallen person is gone").

Run from PowerShell:
    C:\isaacsim\python.bat C:\isaacsim\projects\surveillance-proj\diagnose_fall_translation.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdSkel

PATH = r"C:\isaacsim\projects\surveillance-proj\assets\character.usd"


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(PATH)
    for _ in range(30):
        simulation_app.update()
    stage = ctx.get_stage()

    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdSkel.Animation) and prim.GetName() in ("mixamo_com", "mixamo_fall"):
            anim = UsdSkel.Animation(prim)
            trans_attr = anim.GetTranslationsAttr()
            rot_attr = anim.GetRotationsAttr()
            samples = trans_attr.GetTimeSamples()
            rot_samples = rot_attr.GetTimeSamples()
            print(f"\n{prim.GetPath()}")
            print(f"  time range (rotations): {rot_samples[0]} to {rot_samples[-1]} ({len(rot_samples)} samples)")
            print(f"  time range (translations): {samples[0] if samples else 'N/A'} to {samples[-1] if samples else 'N/A'} ({len(samples)} samples)")
            if samples:
                first_root = trans_attr.Get(samples[0])[0]
                last_root = trans_attr.Get(samples[-1])[0]
                print(f"  root joint[0] translation FIRST sample: {first_root}")
                print(f"  root joint[0] translation LAST sample:  {last_root}")
                dx = last_root[0] - first_root[0]
                dy = last_root[1] - first_root[1]
                dz = last_root[2] - first_root[2]
                print(f"  net drift: dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} (raw units, x0.01 for meters after model scale)")

    print("\nDONE.")


if __name__ == "__main__":
    main()
    simulation_app.close()
