# Python upgrade instructions

md_content = """# Upgrade Python to 3.11 on Jetson Nano

This guide provides step-by-step instructions to upgrade Python to version **3.11** on Jetson Nano and set it as the **default version**.

## 🚀 Step 1: Install Required Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common build-essential \
    zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev \
    libssl-dev libreadline-dev libffi-dev libsqlite3-dev \
    libbz2-dev curl wget
cd /usr/src
sudo wget https://www.python.org/ftp/python/3.11.2/Python-3.11.2.tgz
sudo tar xzf Python-3.11.2.tgz
cd Python-3.11.2
sudo ./configure --enable-optimizations
sudo make -j$(nproc)



sudo make altinstall
sudo update-alternatives --install /usr/bin/python3 python3 /usr/local/bin/python3.11 1
sudo update-alternatives --config python3

curl -sS https://bootstrap.pypa.io/get-pip.py | python3
pip3 install --upgrade setuptools wheel cython packaging

sudo ln -sf /usr/local/bin/python3.11 /usr/bin/python3
