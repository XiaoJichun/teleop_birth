# pip install huggingface-hub --upgrade
# pip install chardet charset-normalizer

from huggingface_hub import login, snapshot_download, whoami

import os
import sys

os.environ["LANG"] = "en_US.UTF-8"
os.environ["LC_ALL"] = "en_US.UTF-8"
sys.stdout.reconfigure(encoding='utf-8')

# 👇 只加这一行，临时加速，不污染环境
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"



# ===================== 配置区域 =====================
HF_TOKEN = ""
# ====================================================

# 登录并校验是否成功
try:
    # 执行登录
    login(token=HF_TOKEN)
    # 校验登录状态
    user_info = whoami()
    print(f"✅ 登录成功！用户：{user_info['name']}")
except Exception as e:
    print(f"❌ 登录失败：{str(e)}")
    print("将以未登录模式继续下载（速度可能较慢）")

# 下载 DiT4DiT 模型
print("\n开始下载 mondo-robotics/dit4dit-model 模型...")
snapshot_download(
    repo_id="mondo-robotics/dit4dit-model",
    repo_type="model",
    local_dir="/root/workspace/DiT4DiT/checkpoints/dit4dit-model",
    allow_patterns=["dit4dit_robocasa_gr1/**/*"],  # 只下载你需要的文件夹
    max_workers=16,
    resume_download=True,
)

print("\n🎉 DiT4DiT 模型下载完成！")
