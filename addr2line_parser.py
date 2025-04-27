import subprocess

def parse_backtrace(elf_path, addresses):
    cmd = ["xtensa-esp32-elf-addr2line", "-e", elf_path] + addresses
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(result.stdout)

def main():
    elf_path = input("请输入 ELF 文件路径（比如 build/your_app.elf）: ").strip()
    bt_input = input("请输入 Backtrace 地址串（空格分隔）: ").strip()
    addresses = bt_input.split()
    parse_backtrace(elf_path, addresses)

if __name__ == "__main__":
    main()
