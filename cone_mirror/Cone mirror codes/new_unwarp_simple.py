import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('./test.png')

h, w = img.shape[:2]
cx, cy = w // 2, h // 2
radius = int((h / 2.0) * 0.75)

angle_samples = 2048          # 360° 方向采样数（对应长边）
radial_samples = radius       # 半径方向采样数（短边）

# 1. 圆形图像 -> 极坐标展开（就是你说的 unwarp）
#    dsize = (width, height)，width 对应角度，height 对应半径
polar = cv2.warpPolar(
    img,
    (int(angle_samples), int(radial_samples)),  # (宽=角度360°, 高=半径)
    (float(cx), float(cy)),
    float(radius),
    cv2.WARP_POLAR_LINEAR
)
# polar.shape == (radial_samples, angle_samples, 3)
# axis=0: 半径 r（短边）
# axis=1: 角度 θ（长边 = 360°）

# 2. 在“长边 = 360°”方向上平移 90°
deg_offset = 90               # 想反方向就改成 -90
shift = int(angle_samples * deg_offset / 360.0)

# 沿 axis=1（角度轴 = 长边）做环形平移
polar_shifted = np.roll(polar, shift=shift, axis=1)

# 3. 这一张就是你要的最终 unwarp 结果（不再 warp 回圆形）
plt.imshow(cv2.cvtColor(polar_shifted, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()
