import matplotlib.pyplot as plt
from matplotlib import font_manager
import cv2
import numpy as np
import os


# ================================
# 修复中文字体显示问题
# ================================
def setup_chinese_font():
    """设置中文字体支持"""
    # 方法1: 使用常见中文字体名称
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'KaiTi', 'FangSong']
    plt.rcParams['axes.unicode_minus'] = False

    # 方法2: 如果上述方法无效，使用英文标题
    plt.rcParams['font.family'] = 'sans-serif'


def robust_image_reader(file_path):
    """健壮的图像读取函数"""
    print(f"正在读取: {file_path}")

    if not os.path.exists(file_path):
        print("❌ 文件不存在")
        return None

    # 使用imdecode读取（解决中文路径问题）
    try:
        with open(file_path, 'rb') as f:
            img_array = np.frombuffer(f.read(), dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            print("✅ OpenCV imdecode读取成功")
            return img
    except Exception as e:
        print(f"读取失败: {e}")

    return None


def watershed_segmentation_complete():
    """完整的分水岭分割算法"""
    # 设置中文字体
    setup_chinese_font()

    # 文件路径
    file_path = r"C:\Users\Y9000\PycharmProjects\multirotor\Screenshot 2026-01-22 105627.png"

    # 读取图像
    img = robust_image_reader(file_path)
    if img is None:
        return None

    print(f"✅ 图像读取成功，尺寸: {img.shape}")

    # 基于文档的分水岭算法完整步骤
    # Step1. 阈值分割（Otsu大津法）
    ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    print(f"Otsu阈值: {ret}")

    # Step2. 开运算去噪
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # Step3. 确定背景区域
    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    # Step4. 距离变换获取前景
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(dist_transform, 0.1 * dist_transform.max(), 255, 0)

    # Step5. 确定未知区域
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # Step6. 连通区域处理
    ret, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    # Step7. 分水岭算法
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(img_color, markers)

    # 创建结果可视化
    img_result = img_color.copy()
    img_result[markers == -1] = [0, 0, 255]  # 边界标记为红色

    # 使用英文标题避免字体问题
    titles = ['Original Image', 'Threshold', 'Opening', 'Background',
              'Distance Transform', 'Foreground', 'Unknown Region',
              'Watershed Result']

    # 显示所有步骤
    images = [img, thresh, opening, sure_bg, dist_transform, sure_fg, unknown, img_result]

    plt.figure(figsize=(20, 10))
    for i in range(8):
        plt.subplot(2, 4, i + 1)
        if i == 4:  # 距离变换图用jet色彩
            plt.imshow(images[i], cmap='jet')
        else:
            plt.imshow(images[i], cmap='gray')
        plt.title(titles[i], fontsize=10)
        plt.axis('off')

    plt.tight_layout()
    plt.show()

    # 显示最终分割结果
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.imshow(img, cmap='gray')
    plt.title('Original Grayscale Image')
    plt.axis('off')

    plt.subplot(1, 3, 2)
    plt.imshow(cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB))
    plt.title('Watershed Segmentation')
    plt.axis('off')

    plt.subplot(1, 3, 3)
    # 显示区域统计
    plt.axis('off')
    regions_count = ret - 1
    stats_text = f"Segmentation Results:\nRegions: {regions_count}\nImage Size: {img.shape}\nBoundary Pixels: {np.sum(markers == -1)}"
    plt.text(0.1, 0.5, stats_text, fontsize=12, transform=plt.gca().transAxes,
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

    plt.tight_layout()
    plt.show()

    print(f"✅ 分割完成！识别出 {regions_count} 个区域")
    return markers, img_result


if __name__ == "__main__":
    print("分水岭算法分割程序 - 修复版")
    print("=" * 50)
    watershed_segmentation_complete()