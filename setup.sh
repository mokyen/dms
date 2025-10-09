#!/usr/bin/env bash

set -e

# 1. Update package lists and install Python 3 and venv
sudo apt update
sudo apt install -y python3 python3-venv

# 2. Download and run the PlatformIO installer
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

# 3. Add pio shortcut to ~/.local/bin
mkdir -p ~/.local/bin
ln -sf ~/.platformio/penv/bin/platformio ~/.local/bin/pio

# 4. Ensure ~/.local/bin is in PATH for current and future shells
if ! echo "$PATH" | grep -q "$HOME/.local/bin"; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    export PATH="$HOME/.local/bin:$PATH"
fi

echo "PlatformIO setup complete! Open a new terminal or run 'source ~/.zshrc' or 'source ~/.bashrc' if needed."
echo "You can now use 'pio' from the command line."