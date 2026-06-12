# STM32 USART Bootloader GUI Tool

## 📌 Overview
Python-based GUI tool to flash STM32F767 using USART bootloader (AN3155 protocol).

## 🚀 Features
- Supports Intel HEX & SREC
- Mass erase + write + verify
- Live progress tracking
- UART communication (pyserial)
- Tkinter GUI

## 🧰 Hardware Setup
- BOOT0 → HIGH
- USART3:
  - PB10 → RX (USB UART)
  - PB11 → TX
- Use CP2102 / FT232

## ⚙️ Installation
```bash
pip install -r requirements.txt

## How to Run
- After installing the latest version of python just run this file in cmd prompt
- Right click open with python default or CMD(run as administartor)


