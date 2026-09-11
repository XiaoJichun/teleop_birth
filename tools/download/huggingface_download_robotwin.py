# pip install huggingface-hub --upgrade
# pip install chardet charset-normalizer
from huggingface_hub import login, snapshot_download, whoami
import os
import sys

os.environ["LANG"] = "en_US.UTF-8"
os.environ["LC_ALL"] = "en_US.UTF-8"
sys.stdout.reconfigure(encoding='utf-8')
# 👇 HF镜像加速，临时环境变量
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"

# ===================== 配置区域 =====================
HF_TOKEN = ""
# 数据集repo_id
DATASET_REPO_ID = "ganlinyang/RoboTwin-MeM"
# 需要下载的子目录：lerobot_2.1
SUB_FOLDER = "lerobot_2.1"
# 本地保存路径，按需修改
LOCAL_DIR = "/root/workspace/vla_repo/vla_data_repo/training_data/Lerobot/robotwin_joint"
# ====================================================

# 登录并校验是否成功
try:
    login(token=HF_TOKEN)
    user_info = whoami()
    print(f"✅ HuggingFace登录成功！用户：{user_info['name']}")
except Exception as e:
    print(f"⚠️ 登录失败：{str(e)}")
    print("将以未登录模式继续下载数据集（部分私有数据集会受限）")

print(f"\n开始下载数据集 {DATASET_REPO_ID} 子目录 {SUB_FOLDER} ...")
snapshot_download(
    repo_id=DATASET_REPO_ID,
    repo_type="dataset",   # 重点：dataset，不是model
    local_dir=LOCAL_DIR,
    local_dir_use_symlinks=False,  # 数据集不要符号链接，直接落地真实文件
    allow_patterns=[
        f"{SUB_FOLDER}/**/*"   # 只拉取 lerobot_2.1 下全部文件，不下载仓库其他内容
    ],
    max_workers=16,
    resume_download=True,
)

print("\n🎉 RoboTwin‑MeM lerobot_2.1 数据集下载完成！")
print(f"📂 本地路径：{LOCAL_DIR}")
