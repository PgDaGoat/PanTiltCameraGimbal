STM32 YOLO Gimbal Tracking - Complete System

Working Trackers:

1. [yolo_tracking_simple.py] - Simple & Proven
- Character-by-character serial (fixes STM32L0 limitation)
- Inverted tilt (correct servo direction)
- Single-threaded, straightforward
- ~15 FPS, 1.67 Hz servo updates
- Status: FULLY WORKING ✅

2. [yolo_tracking_advanced.py] - Advanced
- Everything from final PLUS:
- Multi-threaded (60+ FPS display)
- Kalman filtering (smooth tracking)
- Adaptive ROI (2x faster after lock)
- Enhanced PID control
- Occlusion handling
- Status: FULLY WORKING ✅

---

Quick Start

Option 1: Simple Tracking
```bash
python yolo_tracking_simple.py
```
- Straightforward
- Proven to work
- Good for learning

Option 2: Advanced Tracking
```bash
python yolo_tracking_advanced.py
```
- Smoother display (60 FPS)
- Faster tracking (adaptive ROI)
- Professional features
- Still uses your serial solution!

Both work perfectly with your STM32L0! ✅

---

Hardware Configuration

My Setup:**
- **MCU:** STM32L0 Nucleo @ 2.097 MHz
- **Pan Servo:** PA0 (TIM2_CH1)
- **Tilt Servo:** PA1 (TIM2_CH2)
- **Serial:** COM3 @ 115200 baud
- **Camera:** USB webcam (index 0)

**Both scripts are pre-configured for this!**

---

Key Discoveries

1. The Tilt Problem
**Symptom:** Pan worked, tilt didn't (in Python)  
**Cause:** Burst serial sending too fast for STM32L0  
**Solution:** Character-by-character with 10ms delays

### 2. The Serial Limitation
**Issue:** STM32L0 @ 2 MHz can't process serial bursts at 115200 baud  
**Why:** Bytes arrive faster than interrupt handler can process  
**Fix:** Send each character with 10ms delay (mimics PuTTY typing)

### 3. The Direction Issue
**Symptom:** Tilt worked but reversed  
**Solution:** Invert tilt angle calculation (180 - angle)

---

## 📊 Performance Comparison

| Metric | Final | Advanced |
|--------|-------|----------|
| Display FPS | 15 | 60+ |
| Tracking FPS | 2.5 (full) | 15-30 (adaptive) |
| Servo Update | 1.67 Hz | 1.67 Hz |
| Smoothness | Good | Excellent |
| Complexity | Low | Medium |
| Features | Core | Full |

---

## 🛠️ Troubleshooting

### Tilt Not Working?
1. Check serial port: COM3
2. Verify char-by-char sending enabled
3. Check PA1 connection
4. Test with: `python test_tilt_servo.py`

### Slow Performance?
1. Check FPS metrics on display
2. Lower YOLO resolution (imgsz=320)
3. Use adaptive version for 2x speedup

### Servos Jittery?
1. Increase command delay (0.8-1.0s)
2. Check power supply to servos
3. Advanced version has smoothing built-in

---

## 🎯 Recommendations

**For Learning:**
- Start with `yolo_tracking_final.py`
- Understand the basics
- Then try advanced

**For Production:**
- Use `yolo_tracking_advanced.py`
- Smoother, faster, more professional
- Still simple to run!

---

## 💡 What Was Fully Built

✅ Full 2-axis object tracking gimbal  
✅ YOLO-powered detection  
✅ STM32 servo control  
✅ Character-by-character serial  
✅ Multiple tracking modes  
✅ Kalman filtering (advanced)  
✅ Adaptive ROI (advanced)  
✅ Professional display  

---

## 📈 Next Steps

### Improvements I would make in the future:
1. **Faster YOLO:** Try yolo11n.pt (no segmentation)
2. **Higher Resolution:** Change imgsz=640 (slower but more accurate)
3. **Custom PID:** Tune parameters in advanced version
4. **Multiple Objects:** Modify to track multiple people

### Hardware Upgrades (Optional):
1. **STM32F4:** Would allow burst serial (faster)
2. **DMA UART:** Would allow higher baud rates
3. **Better Servos:** Faster response time


---

## 🎓 Key Lessons

1. **Slow MCUs need slow serial** - Char-by-char was the key insight
2. **PuTTY vs Python difference** - Manual typing vs burst sending
3. **Multi-threading improves UX** - 60 FPS display even with slow YOLO
4. **Kalman filtering smooths tracking** - Better than raw detections
5. **Adaptive ROI saves CPU** - Only scan where you need to

---

## 🏆 Success Metrics

**What was Achieved:**
- ✅ Debugged complex serial issue
- ✅ Found STM32L0 processing limit
- ✅ Created char-by-char solution
- ✅ Fixed inverted tilt
- ✅ Built working tracker
- ✅ Enhanced with advanced features



