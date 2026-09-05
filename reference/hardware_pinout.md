# 硬體 Pinout — Waveshare ESP32-S3-LCD-1.28（Non-Touch 版本）

> 這是給 agent／自己寫 driver code 時查的 ground truth，獨立於 `gravity_timer_project_plan.md`，避免每次都要重新爬整份計畫文件。原始來源見文末，本機備份在 [`vendor/`](vendor/)。

> ⚠️ 版本注意：本專案用的是 **Non-Touch 版**（ESP32-S3-LCD-1.28），不是 Touch 版（ESP32-S3-Touch-LCD-1.28）。兩者 pin 定義不同，不可混用網路上找到的 Touch 版範例程式的 pin config。

## Pin 對照

已對照官方 wiki pin table 與 schematic 交叉確認一致。

| Component | GPIO | 備註 |
|---|---|---|
| LCD_BL | 40 | |
| LCD_DC | 8 | |
| LCD_CS | 9 | |
| LCD_CLK | 10 | |
| LCD_MOSI (LCD_DIN) | 11 | |
| LCD_RST | 12 | |
| I2C_SDA | 6 | |
| I2C_SCL | 7 | |
| IMU_INT1 | 47 | **不是 RTC-capable GPIO**（2026-09-05 查 ESP-IDF `soc_caps.h`/`rtc_io_channel.h` 確認 ESP32-S3 的 RTC domain 只涵蓋 GPIO0-21），deep sleep 期間的 `ext0`/`ext1` 喚醒源、ULP 協同處理器都碰不到這支腳，只有 light sleep 的 GPIO 喚醒（不受 RTC domain 限制）用得上，見 `gravity_timer_project_plan.md` M9 |
| IMU_INT2 | 48 | 同上，不是 RTC-capable GPIO |
| Battery ADC | 1 | 分壓 R4=R7=100K（1:1），電壓 = 3.3/4096×2×AD_Value；R7 並聯 C13 100nF 做濾波（2026-08-25 直接對照 schematic 修正，原先誤植為 200K:100K/×3） |
| BOOT0 | 0 | |
| UART_TXD / UART_RXD | 43 / 44 | 經 CH343P 轉 USB，燒錄與 log 用，不需自行接線 |

本版本 schematic 上**沒有觸控 IC（CST816S）**，I2C bus 上僅 QMI8658 一個裝置，不需擔心位址衝突或匯流排共用問題。IMU_INT1/INT2 也是獨立腳位，沒有與其他功能共腳的疑慮。

## 動手前建議確認

- 官方 wiki 的 Datasheet 連結指向 **QMI8658A**，但 schematic 上實際料號標示為 **QMI8658C**（U4）。兩者暫存器多半相容，但 tap engine 細部參數（Section 10）可能因版本略有出入，設定時建議兩份 datasheet 交叉對照，並以實測結果為準。
  - **2026-09-05 更新**：已取得正確料號的 **QMI8658C** datasheet（Rev A，QST 官方，`vendor/QMI8658C.pdf`），不再需要靠 QMI8658A 版本猜測——之前 `calibration_mode.cpp`/`qmi8658.hpp` 裡對照 gFS 表用的 "Rev 0.6" 是社群流通的舊版本號，跟這份 Rev A 內容一致，暫存器位置與現有程式碼對得上，不需要重新校對。Table 15/16/17（Current Consumption）是 M9 評估 IMU ODR 省電空間時的依據，見 `gravity_timer_project_plan.md` M9。

## 關鍵晶片能力

- **QMI8658 內建硬體 Tap Detection Engine**（連同 Any-Motion / No-Motion / Significant-Motion / Pedometer），透過 CTRL9 指令設定。本專案僅需 single-tap 判斷（無雙拍），設定複雜度較低。
- SPI 支援 80MHz，240×240 全螢幕 flush 理論 ~11.5ms，不是明顯瓶頸，partial buffer 仍是好習慣但非必要。
- ESP32-S3R2：2MB PSRAM（封裝內建）＋ 16MB Flash ＋ 240MHz 雙核，雙緩衝 framebuffer（~230KB）綽綽有餘。
- 官方僅提供 Arduino / MicroPython demo，無 ESP-IDF 範例，pin init / driver 需自行從 Arduino demo 移植邏輯。

## 原始來源

- [Waveshare Wiki（Non-Touch 版本）](https://www.waveshare.com/wiki/ESP32-S3-LCD-1.28) — 本機備份：[`vendor/waveshare_esp32-s3-lcd-1.28_wiki.html`](vendor/waveshare_esp32-s3-lcd-1.28_wiki.html)
- [官方 Schematic PDF](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.28/Esp32-s3-lcd-.128-sch.pdf) — 本機備份：[`vendor/esp32-s3-lcd-1.28_schematic.pdf`](vendor/esp32-s3-lcd-1.28_schematic.pdf)
- [QMI8658A Datasheet（官方連結，注意板上實際料號為 QMI8658C，設定 tap 參數時建議交叉核對）](https://files.waveshare.com/wiki/common/QMI8658A_Datasheet_Rev_A.pdf)
- [GC9A01A Datasheet](https://files.waveshare.com/wiki/common/GC9A01A.pdf)
- [ESP32-S3 Datasheet](https://files.waveshare.com/wiki/common/Esp32-s3_datasheet_en.pdf)
- [ESP32-S3 Technical Reference Manual](https://files.waveshare.com/wiki/common/Esp32-s3_technical_reference_manual_en.pdf)
