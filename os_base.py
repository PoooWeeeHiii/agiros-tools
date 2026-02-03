
import os
import subprocess
    


def get_sys_info():
    """
    获取当前系统的信息，包括操作系统名称、版本、版本代号和架构。
    要求：
    - 支持Ubuntu、Debian、Fedora、CentOS、openEuler等常见Linux发行版。
    - 能够获取操作系统名称、版本、版本代号（如 jammy, noble）和架构（如 amd64, arm64）。
    - 能够获取python版本（如 3.10, 3.11, 3.12）。
    """
    # 初始化默认值
    os_name = "Unknown"
    os_version = "Unknown"
    os_codename = "Unknown"
    
    # 获取系统架构
    try:
        result = subprocess.run(["uname", "-m"], capture_output=True, text=True, check=True)
        os_arch = result.stdout.strip()
    except:
        os_arch = "Unknown"
    
    # 获取Python版本
    try:
        import sys
        python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    except:
        python_version = "Unknown"
    
    # 尝试从/etc/os-release获取系统信息（适用于大多数现代Linux发行版）
    try:
        with open("/etc/os-release", "r") as f:
            for line in f:
                if line.startswith("NAME="):
                    os_name = line.split("=")[1].strip('"')
                elif line.startswith("VERSION_ID="):
                    os_version = line.split("=")[1].strip('"')
                elif line.startswith("VERSION_CODENAME="):
                    os_codename = line.split("=")[1].strip('"')
                elif line.startswith("PRETTY_NAME="):
                    # 备用方案：从PRETTY_NAME中提取信息
                    pretty_name = line.split("=")[1].strip('"')
                    if os_name == "Unknown":
                        os_name = pretty_name.split()[0]
    except:
        pass
    
    # 特殊处理某些发行版
    # 处理CentOS/RHEL
    if os_name.lower() in ["centos", "red hat", "red hat enterprise linux", "rhel"]:
        try:
            result = subprocess.run(["cat", "/etc/centos-release"], capture_output=True, text=True)
            output = result.stdout.strip()
            if output:
                # 尝试从centos-release文件中提取更准确的版本信息
                parts = output.split()
                for i, part in enumerate(parts):
                    if part.replace(".", "").isdigit():
                        os_version = part
                        break
        except:
            pass
    # 处理Debian/Ubuntu
    elif os_name.lower() in ["debian", "ubuntu"]:
        # 尝试从/etc/lsb-release获取更详细的代号信息
        try:
            with open("/etc/lsb-release", "r") as f:
                for line in f:
                    if line.startswith("DISTRIB_CODENAME="):
                        os_codename = line.split("=")[1].strip('"')
        except:
            pass
    # 处理openEuler
    elif "openeuler" in os_name.lower():
        try:
            result = subprocess.run(["cat", "/etc/openEuler-release"], capture_output=True, text=True)
            output = result.stdout.strip()
            if output:
                # 尝试从openEuler-release文件中提取更准确的版本信息
                parts = output.split()
                for part in parts:
                    if part.replace(".", "").isdigit():
                        os_version = part
                        break
        except:
            pass
    
    # 如果仍然没有获取到代号，尝试从/etc/debian_version获取（适用于Debian系）
    if os_codename == "Unknown" and os_name.lower() in ["debian", "ubuntu"]:
        try:
            result = subprocess.run(["lsb_release", "-c"], capture_output=True, text=True)
            os_codename = result.stdout.split(":")[1].strip()
        except:
            pass
    #对os_name, os_version, os_codename, os_arch, python_version的字符串，判断后面是否由\n,如果有，则去掉\n
    os_name = os_name.replace('"\n', "")
    os_version = os_version.replace('"\n', "")
    os_codename = os_codename.replace('\n', "")
    #os_arch = os_arch.replace('"\n', "")
    #python_version = python_version.replace('"\n', "")
    return os_name, os_version, os_codename, os_arch, python_version