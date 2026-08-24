"""把 PNP7DataConfig 从单相机切换到双相机。"""
p = "/home/andyls/vla/RLinf/rlinf/models/embodiment/openpi/dataconfig/pnp7_dataconfig.py"
s = open(p).read()

s = s.replace(
    "from rlinf.models.embodiment.openpi.policies import franka_policy",
    "from rlinf.models.embodiment.openpi.policies import pnp7_policy",
)

s = s.replace(
    '                        "observation/image": "image",\n'
    '                        "observation/state": "state",',
    '                        "observation/image": "image",\n'
    '                        "observation/wrist_image": "wrist_image",\n'
    '                        "observation/state": "state",',
)

s = s.replace(
    "                franka_policy.FrankaEEInputs(\n"
    "                    action_dim=model_config.action_dim,\n"
    "                    model_type=model_config.model_type,\n"
    "                )",
    "                pnp7_policy.PNP7Inputs(\n"
    "                    action_dim=model_config.action_dim,\n"
    "                    model_type=model_config.model_type,\n"
    "                )",
)

s = s.replace(
    "                franka_policy.FrankaEEOutputs(\n"
    "                    output_action_dim=self.output_action_dim\n"
    "                )",
    "                pnp7_policy.PNP7Outputs(\n"
    "                    output_action_dim=self.output_action_dim\n"
    "                )",
)

old_doc = (
    "Dataset schema:\n"
    "  state   [x, y, z, rx, ry, rz, gripper]     absolute pose + normalised opening\n"
    "  actions [dx, dy, dz, drx, dry, drz, grip]  pose delta + absolute gripper\n"
    '"""'
)
new_doc = (
    "Dataset schema:\n"
    "  state       [x,y,z, r00,r10,r20, r01,r11,r21, gripper]  pose + 6D rotation\n"
    "  actions     [dx,dy,dz, drx,dry,drz, gripper]            delta + gripper 0/1\n"
    "  image       external camera\n"
    "  wrist_image wrist camera\n"
    "\n"
    "Uses PNP7Inputs rather than FrankaEEInputs so BOTH cameras reach the model;\n"
    "FrankaEEInputs zeroes and masks off the wrist slots, so the recorded wrist\n"
    "view never reaches the network.\n"
    '"""'
)
if old_doc in s:
    s = s.replace(old_doc, new_doc)

open(p, "w").write(s)

import re
assert "pnp7_policy.PNP7Inputs" in s, "Inputs 未替换"
assert "pnp7_policy.PNP7Outputs" in s, "Outputs 未替换"
assert '"observation/wrist_image": "wrist_image"' in s, "repack 未加腕部图"
assert "franka_policy" not in s, "仍残留 franka_policy 引用"
print("dataconfig 已切换到双相机 transform，全部断言通过")
