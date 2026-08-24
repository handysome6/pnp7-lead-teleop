"""验证腕部相机确实进入了模型输入，而不是被静默忽略。

只跑 transform，不碰 GPU，可以和训练并行执行。
"""
import sys

import numpy as np

sys.path.insert(0, "/workspace/RLinf")

from openpi.models import model as _model

from rlinf.models.embodiment.openpi.policies import pnp7_policy

H = W = 224
# 两路图给成完全可区分的常数，便于判断哪一路落到了哪个槽位
base = np.full((H, W, 3), 11, dtype=np.uint8)
wrist = np.full((H, W, 3), 222, dtype=np.uint8)

tf = pnp7_policy.PNP7Inputs(action_dim=32, model_type=_model.ModelType.PI0)
out = tf({
    "observation/state": np.zeros(10, dtype=np.float32),
    "observation/image": base,
    "observation/wrist_image": wrist,
    "actions": np.zeros((10, 7), dtype=np.float32),
    "prompt": "pick up the blue cube and place it in the plate",
})

print("%-20s %-12s %-8s %s" % ("槽位", "均值", "mask", "判定"))
expect = {"base_0_rgb": 11, "left_wrist_0_rgb": 222, "right_wrist_0_rgb": 0}
ok = True
for name in ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"):
    img = np.asarray(out["image"][name])
    m = bool(out["image_mask"][name])
    mean = float(img.mean())
    good = abs(mean - expect[name]) < 1.0
    ok &= good
    verdict = {
        "base_0_rgb": "外部相机" if good else "错!",
        "left_wrist_0_rgb": "腕部相机" if good else "错!",
        "right_wrist_0_rgb": "空(预期)" if good else "错!",
    }[name]
    print("%-20s %-12.1f %-8s %s" % (name, mean, m, verdict))

masks = [bool(out["image_mask"][n]) for n in
         ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")]
print()
print("mask 期望 [True, True, False]，实际", masks)
print("state 维度(补齐后):", tuple(np.asarray(out["state"]).shape))
print("actions 维度(补齐后):", tuple(np.asarray(out["actions"]).shape))
print()
if ok and masks == [True, True, False]:
    print("通过：两路相机都真实进入模型，腕部图位于 left_wrist_0_rgb 且未被 mask")
else:
    print("失败：图像槽位或 mask 不符合预期")
    sys.exit(1)
