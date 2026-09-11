# ===================== 自动安装必需库（只装 huggingface-hub） =====================
import subprocess
import sys
import os

# 只在找不到库时安装
try:
    from huggingface_hub import login, snapshot_download, whoami
except ImportError:
    print("正在自动安装 huggingface-hub ...")
    subprocess.check_call([
        sys.executable, "-m", "pip", "install", "--upgrade", "huggingface-hub"
    ])
    from huggingface_hub import login, snapshot_download, whoami

# ===================== 配置 =====================
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"

# ===================== 下载配置 =====================
HF_TOKEN   = ""
NAME       = "robbyant/lingbot-va-posttrain-robotwin"
LOCAL_DIR  = "/root/workspace/vla_repo/vla_data_repo/model_data/lingbot-va-posttrain-robotwin"
REPO_TYPE  = "model"
MAX_WORKERS = 16

# 登录
try:
    login(token=HF_TOKEN)
    print("✅ 登录成功")
except Exception as e:
    print(f"❌ 登录失败：{e}")

# ===================== 核心：判断模型是否完整下载 =====================
def is_model_complete(local_dir):
    """
    只有满足：
    1. 目录存在
    2. 没有任何 .incomplete 临时文件
    才算【完整下载】
    """
    if not os.path.exists(local_dir):
        return False

    # 检查是否有未下载完的临时文件
    for root, dirs, files in os.walk(local_dir):
        for file in files:
            # 只要有临时文件 = 不完整
            if file.endswith(".incomplete"):
                return False
    return True

# ===================== 执行下载逻辑 =====================
if is_model_complete(LOCAL_DIR):
    print(f"\n✅ 模型已【完整下载】，路径：{LOCAL_DIR}，跳过下载")
else:
    if os.path.exists(LOCAL_DIR):
        print(f"\🔄 模型不完整，开始【断点续传】...")
    else:
        print(f"\n📥 模型不存在，开始【全新下载】...")

    # 执行下载（自带断点续传）
    snapshot_download(
        repo_id=NAME,
        repo_type=REPO_TYPE,
        local_dir=LOCAL_DIR,
        max_workers=MAX_WORKERS,
        resume_download=True,  # 断点续传开关
    )
    print("\n🎉 下载完成！")
