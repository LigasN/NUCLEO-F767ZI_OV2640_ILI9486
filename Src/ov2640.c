/**
 * ov2640.c
 *
 * 	Created on: 23.02.2019
 *	Modified on:  23.02.2021
 *
 *	Copyright 2021 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person
 *obtaining a copy of this software and associated documentation files
 *(the "Software"), to deal in the Software without restriction,
 *including without limitation the rights to use, copy, modify, merge,
 *publish, distribute, sublicense, and/or sell copies of the Software,
 *and to permit persons to whom the Software is furnished to do so,
 *subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be
 *included in all copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 *BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 *ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 ******************************************************************************
 */
#define DEBUG

#include "ov2640.h"
/**
 * Code debugging option
 */
//#define DEBUG

I2C_HandleTypeDef* phi2c;
DCMI_HandleTypeDef* phdcmi;
UART_HandleTypeDef* phuart;

/* Initialization sequence */
const unsigned char InitializationSequence[][2] = {
    {0xff,
     0x00}, /* Switch to device control register list Table 12    */
    {0x2c, 0xff}, /* Reserved */
    {0x2e, 0xdf}, /* Reserved */
    {0xff,
     0x01}, /* Switch to device control register list Table 13    */
    {0x3c, 0x32}, /* Reserved */
    {0x11,
     0x00}, /* CLKRC - Clock Rate Control  RW
                   Bit[7]: Internal frequency doublers
                       0: OFF
                       1: ON
                   Bit[6]: Reserved
                   Bit[5:0]: Clock divider

               CLK = XVCLK/(decimal value of CLKRC[5:0] + 1)
                                                                  */
    {0x09,
     0x02}, /* COM2 - Common control 2  RW
                   Bit[7:5]: Reserved
                   Bit[4]:   Standby mode enable
                       0: Normal mode
                       1: Standby mode
                   Bit[3]:   Reserved
                   Bit[2]:   Pin PWDN/RESETB used as SLVS/SLHS
                   Bit[1:0]: Output drive select
                       00: 1x capability
                       01: 3x capability
                       10: 2x capability
                       11: 4x capability
                                                                  */
    {0x04,
     0b10101000}, /* REGO4 - Register 04  RW
                         Bit[7]:   Horizontal mirror
                         Bit[6]:   Vertical flip
                         Bit[5]:   ??? no documentation here
                         Bit[4]:   VREF bit[0]
                         Bit[3]:   HREF bit[0]
                         Bit[2]:   Reserved
                         Bit[1:0]: AEC[1:0]

                     Info: AEC[15:10] is in register REG45[5:0]
                     (0x45), AEC[9:2] is in register AEC[7:0] (0x10))
                                                                        */
    {0x13,
     0b11100101}, /* COM8 - Common control 8  RW
                         Bit[7:6]: Reserved  --->
                             from VV register Bit[7] = AEC/AGC fast
                     mode Bit[5]: Banding filter selection 0: OFF 1:
                     ON, set minimum exposure time to 1/120s Bit[4:3]:
                     Reserved Bit[2]: | AGC auto/manual control
                     selection 0: Manual 1: Auto Bit[1]: Reserved
                         Bit[O]: Exposure control
                             0: Manual
                             1: Auto
                                                                        */
    {0x14,
     0b01001000}, /* COM9 - Common control 9  RW
                         Bit[7:5]: AGC gain ceiling, GH[2:0]
                             000: 2x
                             001: 4x
                             010: 8x
                             011: 16x
                             100: 32x
                             101: 64x
                             11x: 128x
                         Bit[4:0]: Reserved
                                                                        */
    {0x2c, 0x0c}, /* Reserved */
    {0x33, 0x78}, /* Reserved */
    {0x3a, 0x33}, /* Reserved */
    {0x3b, 0xfB}, /* Reserved */
    {0x3e, 0x00}, /* Reserved */
    {0x43, 0x11}, /* Reserved */
    {0x16, 0x10}, /* Reserved */
    {0x39, 0x02}, /* Reserved */
    {0x35, 0x88}, /* Reserved */
    {0x22, 0x0A}, /* Reserved */
    {0x37, 0x40}, /* Reserved */
    {0x23, 0x00}, /* Reserved */
    {0x34,
     0xa0},       /* ARCOM2 20  RW
                         Bit[7:3]: Reserved
                         Bit[2]:   Zoom window horizontal start point
                         Bit[1:0]: Reserved
                                                                        */
    {0x06, 0x02}, /* Reserved */
    {0x07, 0xc0}, /* Reserved */
    {0x0d,
     0b10110111}, /* COM4 - Common Control 4  RW
                         Bit[7:3]: Reserved
                         Bit[2]: Clock output power-down pin status
                             0: Tri-state data output pin upon
                     power-down 1: Data output pin hold at last state
                     before power-down Bit[1:0]: Reserved
                                                                        */
    {0x0e, 0x01}, /* Reserved */
    {0x4c, 0x00}, /* Reserved */
    {0x4a, 0x81}, /* Reserved */
    {0x21, 0x99}, /* Reserved */
    {0x24,
     0x40}, /* AEW  RW   value = 64(DEC)  default = 0x78 = 120
               Luminance Signal High Range for AEC/AGC Operation
               AEC/AGC values will decrease in auto mode when
               average luminance is greater than AEV[7:0]
                                                                  */
    {0x25,
     0x38}, /* AEB  RW   value = 56(DEC)  default = 0x68 = 104
               Luminance Signal Low Range for AEC/AGC Operation
               AEC/AGC values will increase in auto mode when
               average luminance is less than AEB[7:0]
                                                                  */
    {0x26,
     0x82},       /* VV  RW   value = [High ->8, Low -> 2]
                     Fast Mode Large Step Range Threshold - effective
                     only in AEC/AGC fast mode (COM8[7] = 1)
                         Bit[7:4]: High threshold
                         Bit[3:0]: Low threshold

                     Note: AEC/AGC may change in larger steps when
                           luminance average is greater than VV[7:4]
                           or less than VV[3:0].
                                                                        */
    {0x5c, 0x00}, /* Reserved */
    {0x63, 0x00}, /* Reserved */
    {0x46,
     0x22}, /* FLL - Frame length adjustment  RW
               Frame Length Adjustment LSBs
               Each bit will add 1 horizontal line timing in frame
                                                                  */
    {0x0c,
     0b00111011}, /* COM3 - Common control 3  RW  (before was:
                     00111010) Bit[7:3]: Reserved Bit[2]: Set banding
                     manually 0: 60 Hz 1: 50 Hz Bit[1]: Auto set
                     banding Bit[O]: Snapshot option 0: Enable live
                     video output after snapshot sequence 1: Output
                     single frame only
                                                                        */
    {0x5d,
     0x55}, /* REGSD - Register 5D  RW
               Bit[7:0]: AVGsel[7:0], 16-zone average weight option
                                                                  */
    {0x5e,
     0x7d}, /* REGSD - Register 5E  RW
               Bit[7:0]: AVGsel[15:8], 16-zone average weight
                         option
                                                                  */
    {0x5f,
     0x7d}, /* REGSD - Register 5F  RW
               Bit[7:0]: AVGsel[23:16], 16-zone average weight
                         option
                                                                  */
    {0x60,
     0x55}, /* REGSD - Register 60  RW
               Bit[7:0]: AVGsel[31:24], 16-zone average weight
                         option
                                                                  */
    {0x61,
     0x70}, /* HISTO_LOW  RW             default = 0x80
               Histogram Algorithm Low Level
                                                                  */
    {0x62,
     0x80},       /* HISTO_LOW  RW             default = 0x90
                     Histogram Algorithm High Level
                                                                        */
    {0x7c, 0x05}, /* Reserved */
    {0x20, 0x80}, /* Reserved */
    {0x28, 0x30}, /* Reserved */
    {0x6c, 0x00}, /* Reserved */
    {0x6d, 0x80}, /* Reserved */
    {0x6e, 0x00}, /* Reserved */
    {0x70, 0x02}, /* Reserved */
    {0x71, 0x94}, /* Reserved */
    {0x73, 0xc1}, /* Reserved */
    {0x3d, 0x34}, /* Reserved */
    {0x12,
     0b00000100}, /* COM7 - Common Control 7  RW
                         Bit[7]: SRST
                             1: Initiates system reset. All registers
                     are set to factory default values after which the
                     chip resumes normal operation Bit[6:4]:
                     Resolution selection 000: UXGA (full size) mode
                             001: CIF mode
                             100: SVGA mode
                         Bit[3]: Reserved
                         Bit[2]: Zoom mode
                         Bit[1]: Color bar test pattern
                             0: OFF
                             1: ON
                         Bit[O]: Reserved


                     TODO: Check how ArduCAM has
                                                                        */
    {0x5a, 0x57}, /* Reserved */
    {0x4f,
     0xbb}, /* BD50  RW             default = 0xCA
               50Hz Banding AEC 8 LSBs
                                                                  */
    {0x50,
     0x9c}, /* BD60  RW              default = 0xA8
               60Hz Banding AEC 8 LSBs
                                                                  */
    {0xff,
     0x00}, /* Switch to device control register list Table 12    */
    {0xe5, 0x7f}, /* Reserved */
    {0xf9,
     0b11000000}, /* MC_BIST  RW
                         Bit[7]: Microcontroller Reset
                         Bit[6]: Boot ROM select
                         Bit[5]: R/W 1 error for 12K-byte memory
                         Bit[4]: R/W 0 error for 12K-byte memory
                         Bit[3]: R/W 1 error for 512-byte memory
                         Bit[2]: R/W 0 error for 512-byte memory
                         Bit[1]: BIST busy bit for read; One-shot
                     reset of microcontroller for write Bit[O]: Launch
                     BIST


                     TODO: Check how ArduCAM has
                                                                        */
    {0x41, 0x24}, /* Reserved */
    {0xe0,
     0b00000100}, /* RESET  RW        default = 0b00000100
                     was:0b00010100 Bit[7]: Reserved Bit[6]:
                     Microcontroller Bit[5]: SCCB Bit[4]: JPEG Bit[3]:
                     Reserved Bit[2]: DVP Bit[1]: IPU Bit[O}: CIF
                                                                        */
    {0x76, 0xff}, /* Reserved */
    {0x33, 0xa0}, /* Reserved */
    {0x42, 0x20}, /* Reserved */
    {0x43, 0x18}, /* Reserved */
    {0x4c, 0x00}, /* Reserved */
    {0x87,
     0b11010000}, /* CTRL3  RW                        default =
                     0b01010000 Module Enable Bit[7]:   BPC Bit[6]:
                     WPC Bit[5:0]: Reserved
                                                                        */
    {0x88, 0x3f}, /* Reserved */
    {0xd7, 0x03}, /* Reserved */
    {0xd9, 0x10}, /* Reserved */
    {0xd3,
     0b10000010}, /* R_DVP_SP  RW                     default =
                     0b10000010 Bit[7]:   Auto mode Bit[6:0]: DVP
                     output speed control DVP PCLK = sysclk (48)/[6:0]
                     (YUVO) = sysclk (48)/(2*[6:0]) (RAV)

                     TODO: It needs to be calculated carefully. Check
                     how ArduCAM has
                                                                        */
    {0xc8, 0x08}, /* Reserved */
    {0xc9, 0x80}, /* Reserved */

    // Below it seems like single registers are written more times,
    // but it's SDE. One command gives address and second data. Why? I
    // do not know.

    {0x7c,
     0x00}, /* BPADDR[3:0]  RW                              default =
             0x00 SDE Indirect Register Access: Address
                                                                        */
    {0x7d,
     0x00}, /* BPDATA[7:0]  RW                              default =
             0x00 SDE Indirect Register Access: Data
                                                                        */
    {0x7c,
     0x03}, /* BPADDR[3:0]  RW                              default =
             0x00 SDE Indirect Register Access: Address
                                                                        */
    {0x7d,
     0x48}, /* BPDATA[7:0]  RW                              default =
             0x00 SDE Indirect Register Access: Data
                                                                        */
    {0x7d, 0x48},
    {0x7c,
     0x08}, /* BPADDR[3:0]  RW                              default =
             0x00 SDE Indirect Register Access: Address
                                                                        */
    {0x7d,
     0x20}, /* BPDATA[7:0]  RW                              default =
             0x00 SDE Indirect Register Access: Data
                                                                        */
    {0x7d, 0x10},
    {0x7d, 0x0e},
    {0x90, 0x00}, /* Reserved */
    {0x91, 0x0e}, /* Reserved */
    {0x91, 0x1a}, /* Reserved */
    {0x91, 0x31}, /* Reserved */
    {0x91, 0x5a}, /* Reserved */
    {0x91, 0x69}, /* Reserved */
    {0x91, 0x75}, /* Reserved */
    {0x91, 0x7e}, /* Reserved */
    {0x91, 0x88}, /* Reserved */
    {0x91, 0x8f}, /* Reserved */
    {0x91, 0x96}, /* Reserved */
    {0x91, 0xa3}, /* Reserved */
    {0x91, 0xaf}, /* Reserved */
    {0x91, 0xc4}, /* Reserved */
    {0x91, 0xd7}, /* Reserved */
    {0x91, 0xe8}, /* Reserved */
    {0x91, 0x20}, /* Reserved */
    {0x92, 0x00}, /* Reserved */
    {0x93, 0x06}, /* Reserved */
    {0x93, 0xe3}, /* Reserved */
    {0x93, 0x05}, /* Reserved */
    {0x93, 0x05}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x04}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x93, 0x00}, /* Reserved */
    {0x96, 0x00}, /* Reserved */
    {0x97, 0x08}, /* Reserved */
    {0x97, 0x19}, /* Reserved */
    {0x97, 0x02}, /* Reserved */
    {0x97, 0x0c}, /* Reserved */
    {0x97, 0x24}, /* Reserved */
    {0x97, 0x30}, /* Reserved */
    {0x97, 0x28}, /* Reserved */
    {0x97, 0x26}, /* Reserved */
    {0x97, 0x02}, /* Reserved */
    {0x97, 0x98}, /* Reserved */
    {0x97, 0x80}, /* Reserved */
    {0x97, 0x00}, /* Reserved */
    {0x97, 0x00}, /* Reserved */
    {0xc3,
     0b11101101}, /* CTRL1  RW            default = 0xFF ArduCAM:
                     11101111 Module Enable Bit[7]: CIP Bit[6]: DMY
                             Bit[5]: RAW_GMA
                             Bit[4]: DG
                             Bit[3]: AWB - Automatic White Balance
                             Bit[2]: AWB_GAIN
                             Bit[1]: LENC - LENC color shading
                     correction Bit[O]: PRE
                                                                        */
    {0xa4, 0x00}, /* Reserved */
    {0xa8, 0x00}, /* Reserved */
    {0xc5, 0x11}, /* Reserved */
    {0xc6, 0x51}, /* Reserved */
    {0xbf, 0x80}, /* Reserved */
    {0xc7, 0x10}, /* Reserved */
    {0xb6, 0x66}, /* Reserved */
    {0xb8, 0xA5}, /* Reserved */
    {0xb7, 0x64}, /* Reserved */
    {0xb9, 0x7C}, /* Reserved */
    {0xb3, 0xaf}, /* Reserved */
    {0xb4, 0x97}, /* Reserved */
    {0xb5, 0xFF}, /* Reserved */
    {0xb0, 0xC5}, /* Reserved */
    {0xb1, 0x94}, /* Reserved */
    {0xb2, 0x0f}, /* Reserved */
    {0xc4, 0x5c}, /* Reserved */

    // Size of an image is given to camera on couple of registers
    // below.
    {0xc0, // IMAGE_RESOLUTION_WIDTH >>
     3},   /* HSIZE8[7:0]  RW
              Image Horizontal Size HSIZE[10:3]
              default = 0x80 = 128
              ArduCAM = 0xc8 = 200

TODO: It needs to be calculated carefully
                                               */
    {0xc1, // IMAGE_RESOLUTION_HEIGHT >>
     3},   /* VSIZE8[7:0]  RW
              Image Horizontal Size VSIZE[10:3]
              default = 0x60 = 96
              ArduCAM = 0x96 = 150

TODO: It needs to be calculated carefully
                                               */
    {0x8c,
     //(((IMAGE_RESOLUTION_WIDTH >> 11) & 0b1) << 6) |
     //    ((IMAGE_RESOLUTION_WIDTH & 0b111) << 3) |
     //    (IMAGE_RESOLUTION_HEIGHT &
     0}, // 0b111)},
         /*                  SIZEL[5:0]  (not [6:0]?)  RW
    {HSIZE[11], HSIZE [2:0], VSIZE [2:0]} default =
        0x00 ArduCAM : 0x00

        TODO : It needs to be calculated carefully */
    {0x86,
     0b00111101}, /* CTRL2  RW      default = 0b00001101 ArduCAM:
                     00111101 Module Enable Bit[7:6]: Reserved
                     Bit[5]: DCW Bit[4]:   SDE Bit[3]:   UV_ADJ
                     Bit[2]: UV_AVG Bit[1]:   Reserved Bit[O]: CMX
                                                                        */
    {0x50,
     0b00000000}, /* CTRLI[7:0]  RW           default = 0x00 ArduCAM:
                     0x00 Bit[7]: LP_DP Bit[6]: Round Bit[5:3]:
                     V_DIVIDER Bit[2:0]: H_DIVIDER
                     TODO: I have no idea why we need to divide V and
                     H and if it is a size value or what.
                                                                        */
    {0x51,
     0xC8}, /* HSIZE[7:0]  RW
               default = 64 ArduCAM = 200 skovholm = 144
                   H_SIZE[7:0] (real/4)

               TODO: It needs to be calculated carefully

               TODO: no idea where this value come from as I cannot
                     calculate it from ArduCAM's previous height and
                     width values
                                                                  */
    {0x52,
     0x2c}, /* VSIZE[7:0]  RW
               default = 240 ArduCAM = 150 skovholm = 44
                   V_SIZE[7:0] (real/4)
                   TODO: It needs to be calculated carefully
                                                                  */
    {0x53,
     0x00}, /* XOFFL[7:0]  RW
               default = 0 ArduCAM = 0 skovholm = 0
                   OFFSET_X[7:0]
                                                                  */
    {0x54,
     0x00}, /* YOFFL[7:0]  RW
               default = 0 ArduCAM = 0 skovholm = 0
                   OFFSET_Y[7:0]
                                                                  */
    {0x55,
     0b10001000}, /* VHYX[7:0]  RW
                     default = 0b1000 ArduCAM = 0 skovholm =
                     0b10001000 Bit[7]: V_SIZE[8] Bit[6:4]:
                     OFFSET_Y[10:8] Bit[3]: H_SIZE[8] Bit[2:0]:
                     OFFSET_X[10:8]
                         TODO: It needs to be calculated carefully
                                                                        */
    {0x57,
     0x00}, /* TEST[3:0]  RW    default = 0 ArduCAM = 0 skovholm = 0
                   Bit[7]:   H_SIZE[9]
                   Bit[6:0]: Reserved
                   TODO: It needs to be calculated carefully
                                                                  */
    {0x5a,
     80}, /* ZMOW[7:0]  RW
             default = 88 ArduCAM = 200 skovholm = 80
                 OUTW[7:0] (real/4)
                 TODO: It needs to be calculated carefully
                                                                */
    {0x5b,
     60}, /* ZMOH[7:0]  RW
             default = 72 ArduCAM = 150 skovholm = 60
                 OUTH[7:0] (real/4)
                 TODO: It needs to be calculated carefully
                                                                */
    {0x5c,
     0}, /* ZMHH[1:0]  RW    default = 0 ArduCAM = 0 skovholm = 0
                Bit[7:4]: ZMSPD (zoom speed)
                Bit[2]:   OUTH[8]
                Bit[1:0]: OUTW[9:8]
                                                               */
    {0xd3,
     8}, /* R_DVP_SP  RW
               default = 0b10000010  ArduCAM = 4 || 2  skovholm = 8
                   Bit[7]: Auto mode
                   Bit[6:0]: DVP output speed control
                             DVP PCLK = sysclk (48)/[6:0] (YUVO);
                                      = sysclk (48)/(2*[6:0]) (RAW)
                   TODO: Problematic parameter probably
                                                                  */
    {0xc3,
     0b11101101},   /* CTRL1  RW
                     default = 0xFF ArduCAM: 0b11101101 skovholm:
                     0b11101111 Module Enable Bit[7]: CIP Bit[6]: DMY
                               Bit[5]: RAW_GMA
                               Bit[4]: DG
                               Bit[3]: AWB - Automatic White Balance
                               Bit[2]: AWB_GAIN
                               Bit[1]: LENC - LENC color shading
                     correction Bit[O]: PRE
                           TODO: Check LENC enabled
                                                                          */
    {0x7f, 0x00},   /* Reserved */
    {0xda, 0b1001}, /* IMAGE_MODE  ?RW?
                 default = 0 ArduCAM = 0b00010000 skovholm = 0b1001
                 Image Output Format Select
                     Bit[7]: Reserved
                     Bit[6]: Y8 enable for DVP
                     Bit[5]: Reserved
                     Bit[4]: JPEG output enable
                         0: Non-compressed
                         1: JPEG output
                     Bit[3:2]: DVP output format
                         00: YUV422
                         01: RAW10 (DVP)
                         10: RGB565
                         11: Reserved
                    Bit[1]: HREF timing select in DVP JPEG output mode
                         0: HREF is same as sensor
                         1: HREF = VSYNC
                     Bit[O]: Byte swap enable for DVP
                         0: High byte first YUYV (C2[4] = 0)
                                            YVYU (C2[4] = 1)
                         1: Low byte first UYVY (C2[4] = 0)
                                           VYUY (C2[4] = 1)
                                                                    */
    {0xe5, 0x1f},   /* Reserved */
    {0xe1, 0x67},   /* Reserved */
    {0xe0, 0}, /* RESET  RW   default = 0b0100 ArduCAM = 0 skovholm =
                0 Bit[7]: Reserved Bit[6]: Microcontroller Bit[5]:
                SCCB Bit[4]: JPEG Bit[3]: Reserved Bit[2]: DVP Bit[1]:
                IPU Bit[O}: CIF
                                                               */
    {0xdd, 0x7f}, /* Reserved */
    {0x05, 0}, /* R_BYPASS  RW  default = 1 ArduCAM = 0 skovholm = 0
                Bypass DSP
                    Bit[7:1]: Reserved
                    Bit[O]: Bypass DSP select
                        0: DSP
                        1: Bypass DSP, sensor out directly
                                                               */
    {0x12, 0x40}, /* Reserved */

    // Below are some duplicated register values that I decided to not
    // write again. Left in case that it will broke something
    /*{0xd3, 0x04},        AGAIN R_DVP_SP  RW
                           default = 0b10000010  ArduCAM = 4 || 2
       skovholm = 8
                                                                              */
    /*{0xc0, 0xc8},        AGAIN HSIZE8[7:0]  RW
                       default = 0x80 ArduCAM = 0x16 < different
       skovholm = 0xc8
                                                                              */
    /*{0xc1, 0x96},        AGAIN VSIZE8[7:0]  RW
                       default = 0x60 ArduCAM = 0x12 < different
       skovholm = 0x96
                                                                              */
    /*{0x8c, 0x00},        AGAIN SIZEL[5:0]  (not [6:0]?)  RW
                           default = 0x00 ArduCAM = 0x00 skovholm =
       0x00
                                                                              */
    /*{0x86, 0x3d},        AGAIN CTRL2  RW
                           default = 0xOD ArduCAM = 0x3d skovholm =
       0x35
                                                                              */
    /*{0x50, 0x80},        AGAIN CTRLI[7:0]  RW
                           default = 0x00 ArduCAM = 0x00 skovholm =
       0x80
                                                                              */
    /*{0x51, 0x90},        AGAIN HSIZE[7:0]  RW
                           default = 64 ArduCAM = 44 < different
       skovholm = 144
                                                                              */
    /*{0x52, 0x2c},        AGAIN VSIZE[7:0]  RW
                           default = 240 ArduCAM = 36 < different
       skovholm = 44
                                                                              */
    /*{0x53, 0x00},        AGAIN XOFFL[7:0]  RW
                           default = 0 ArduCAM = 0 skovholm = 0
                                                                              */
    /*{0x54, 0x00},        AGAIN YOFFL[7:0]  RW
                           default = 0 ArduCAM = 0 skovholm = 0
                                                                              */
    /*{0x55, 0b10001000},  AGAIN VHYX[7:0]  RW
                           default = 0b1000 ArduCAM = 0 skovholm =
       0b10001000
                                                                              */
    /*{0x5a, 0x5A},        AGAIN ZMOW[7:0]  RW
                           default = 88 ArduCAM = 44 < different
                           skovholm = 90 < different
                                                                              */
    /*{0x5b, 0x5A},        AGAIN ZMOH[7:0]  RW
                           default = 72 ArduCAM = 36 < different
                           skovholm = 90 < different
                                                                              */
    /*{0x5c, 0x00},        AGAIN ZMHH[1:0]  RW
                           default = 0 ArduCAM = 0 skovholm = 0
                                                                              */

    // Below some additional registers described in ArduCAM code
    {0xc7, 0x40}, /* AGAIN Reserved
                                         ArduCAM = 0x10 skovholm =
                     0x40 0x00  AWB ON 0x40  AWB OFF
                                                                  */
    {0xcc, 0x42}, /* AGAIN Reserved
                                 ArduCAM Home = 0x42 skovholm = 0x5e
                         0x5e  sunny
                                                                  */
    {0xcd, 0x3f}, /* AGAIN Reserved
                                 ArduCAM Home = 0x3f skovholm = 0x41
                                                                  */
    {0xce,
     0x71}, /* AGAIN Reserved
             ArduCAM Home = 0x71 skovholm = 0x54
                                                                  */
    {0xff,
     0xff} /* Switch to device control register list Table 13
                    Something like ending transmission. I do not know
              if that means anything for the camera/
                                                                 */
};

const unsigned char OV2640_JPEG_INIT[][2] = {
    {0xff, 0x00}, {0x2c, 0xff}, {0x2e, 0xdf}, {0xff, 0x01},
    {0x3c, 0x32}, {0x11, 0x00}, {0x09, 0x02}, {0x04, 0x28},
    {0x13, 0xe5}, {0x14, 0x48}, {0x2c, 0x0c}, {0x33, 0x78},
    {0x3a, 0x33}, {0x3b, 0xfB}, {0x3e, 0x00}, {0x43, 0x11},
    {0x16, 0x10}, {0x39, 0x92}, {0x35, 0xda}, {0x22, 0x1a},
    {0x37, 0xc3}, {0x23, 0x00}, {0x34, 0xc0}, {0x36, 0x1a},
    {0x06, 0x88}, {0x07, 0xc0}, {0x0d, 0x87}, {0x0e, 0x41},
    {0x4c, 0x00}, {0x48, 0x00}, {0x5B, 0x00}, {0x42, 0x03},
    {0x4a, 0x81}, {0x21, 0x99}, {0x24, 0x40}, {0x25, 0x38},
    {0x26, 0x82}, {0x5c, 0x00}, {0x63, 0x00}, {0x61, 0x70},
    {0x62, 0x80}, {0x7c, 0x05}, {0x20, 0x80}, {0x28, 0x30},
    {0x6c, 0x00}, {0x6d, 0x80}, {0x6e, 0x00}, {0x70, 0x02},
    {0x71, 0x94}, {0x73, 0xc1}, {0x12, 0x40}, {0x17, 0x11},
    {0x18, 0x43}, {0x19, 0x00}, {0x1a, 0x4b}, {0x32, 0x09},
    {0x37, 0xc0}, {0x4f, 0x60}, {0x50, 0xa8}, {0x6d, 0x00},
    {0x3d, 0x38}, {0x46, 0x3f}, {0x4f, 0x60}, {0x0c, 0x3c},
    {0xff, 0x00}, {0xe5, 0x7f}, {0xf9, 0xc0}, {0x41, 0x24},
    {0xe0, 0x14}, {0x76, 0xff}, {0x33, 0xa0}, {0x42, 0x20},
    {0x43, 0x18}, {0x4c, 0x00}, {0x87, 0xd5}, {0x88, 0x3f},
    {0xd7, 0x03}, {0xd9, 0x10}, {0xd3, 0x82}, {0xc8, 0x08},
    {0xc9, 0x80}, {0x7c, 0x00}, {0x7d, 0x00}, {0x7c, 0x03},
    {0x7d, 0x48}, {0x7d, 0x48}, {0x7c, 0x08}, {0x7d, 0x20},
    {0x7d, 0x10}, {0x7d, 0x0e}, {0x90, 0x00}, {0x91, 0x0e},
    {0x91, 0x1a}, {0x91, 0x31}, {0x91, 0x5a}, {0x91, 0x69},
    {0x91, 0x75}, {0x91, 0x7e}, {0x91, 0x88}, {0x91, 0x8f},
    {0x91, 0x96}, {0x91, 0xa3}, {0x91, 0xaf}, {0x91, 0xc4},
    {0x91, 0xd7}, {0x91, 0xe8}, {0x91, 0x20}, {0x92, 0x00},
    {0x93, 0x06}, {0x93, 0xe3}, {0x93, 0x05}, {0x93, 0x05},
    {0x93, 0x00}, {0x93, 0x04}, {0x93, 0x00}, {0x93, 0x00},
    {0x93, 0x00}, {0x93, 0x00}, {0x93, 0x00}, {0x93, 0x00},
    {0x93, 0x00}, {0x96, 0x00}, {0x97, 0x08}, {0x97, 0x19},
    {0x97, 0x02}, {0x97, 0x0c}, {0x97, 0x24}, {0x97, 0x30},
    {0x97, 0x28}, {0x97, 0x26}, {0x97, 0x02}, {0x97, 0x98},
    {0x97, 0x80}, {0x97, 0x00}, {0x97, 0x00}, {0xc3, 0xed},
    {0xa4, 0x00}, {0xa8, 0x00}, {0xc5, 0x11}, {0xc6, 0x51},
    {0xbf, 0x80}, {0xc7, 0x10}, {0xb6, 0x66}, {0xb8, 0xA5},
    {0xb7, 0x64}, {0xb9, 0x7C}, {0xb3, 0xaf}, {0xb4, 0x97},
    {0xb5, 0xFF}, {0xb0, 0xC5}, {0xb1, 0x94}, {0xb2, 0x0f},
    {0xc4, 0x5c}, {0xc0, 0x64}, {0xc1, 0x4B}, {0x8c, 0x00},
    {0x86, 0x3D}, {0x50, 0x00}, {0x51, 0xC8}, {0x52, 0x96},
    {0x53, 0x00}, {0x54, 0x00}, {0x55, 0x00}, {0x5a, 0xC8},
    {0x5b, 0x96}, {0x5c, 0x00}, {0xd3, 0x00}, {0xc3, 0xed},
    {0x7f, 0x00}, {0xda, 0x00}, {0xe5, 0x1f}, {0xe1, 0x67},
    {0xe0, 0x00}, {0xdd, 0x7f}, {0x05, 0x00}, {0x12, 0x40},
    {0xd3, 0x04}, {0xc0, 0x16}, {0xC1, 0x12}, {0x8c, 0x00},
    {0x86, 0x3d}, {0x50, 0x00}, {0x51, 0x2C}, {0x52, 0x24},
    {0x53, 0x00}, {0x54, 0x00}, {0x55, 0x00}, {0x5A, 0x2c},
    {0x5b, 0x24}, {0x5c, 0x00}, {0xff, 0xff},
};

const unsigned char OV2640_YUV422[][2] = {
    {0xFF, 0x00}, {0x05, 0x00}, {0xDA, 0x10}, {0xD7, 0x03},
    {0xDF, 0x00}, {0x33, 0x80}, {0x3C, 0x40}, {0xe1, 0x77},
    {0x00, 0x00}, {0xff, 0xff},
};

const unsigned char OV2640_JPEG[][2] = {
    {0xe0, 0x14}, {0xe1, 0x77}, {0xe5, 0x1f},
    {0xd7, 0x03}, {0xda, 0x10}, {0xe0, 0x00},
    {0xFF, 0x01}, {0x04, 0x08}, {0xff, 0xff},
};

const unsigned char OV2640_160x120_JPEG[][2] = {
    {0xFF, 0x01}, {0x12, 0x40}, {0x17, 0x11}, {0x18, 0x43},
    {0x19, 0x00}, {0x1a, 0x4b}, {0x32, 0x09}, {0x4f, 0xca},
    {0x50, 0xa8}, {0x5a, 0x23}, {0x6d, 0x00}, {0x39, 0x12},
    {0x35, 0xda}, {0x22, 0x1a}, {0x37, 0xc3}, {0x23, 0x00},
    {0x34, 0xc0}, {0x36, 0x1a}, {0x06, 0x88}, {0x07, 0xc0},
    {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00}, {0xFF, 0x00},
    {0xe0, 0x04}, {0xc0, 0x64}, {0xc1, 0x4b}, {0x86, 0x35},
    {0x50, 0x92}, {0x51, 0xc8}, {0x52, 0x96}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x00}, {0x57, 0x00}, {0x5a, 0x2c},
    {0x5b, 0x24}, {0x5c, 0x00}, {0xe0, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_320x240_JPEG[][2] = {
    {0xff, 0x01}, {0x12, 0x40}, {0x17, 0x11}, {0x18, 0x43},
    {0x19, 0x00}, {0x1a, 0x4b}, {0x32, 0x09}, {0x4f, 0xca},
    {0x50, 0xa8}, {0x5a, 0x23}, {0x6d, 0x00}, {0x39, 0x12},
    {0x35, 0xda}, {0x22, 0x1a}, {0x37, 0xc3}, {0x23, 0x00},
    {0x34, 0xc0}, {0x36, 0x1a}, {0x06, 0x88}, {0x07, 0xc0},
    {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00}, {0xff, 0x00},
    {0xe0, 0x04}, {0xc0, 0x64}, {0xc1, 0x4b}, {0x86, 0x35},
    {0x50, 0x89}, {0x51, 0xc8}, {0x52, 0x96}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x00}, {0x57, 0x00}, {0x5a, 0x50},
    {0x5b, 0x3c}, {0x5c, 0x00}, {0xe0, 0x00}, {0xff, 0xff},
};

const unsigned char OV2640_640x480_JPEG[][2] = {
    {0xff, 0x01}, {0x11, 0x01}, {0x12, 0x00}, {0x17, 0x11},
    {0x18, 0x75}, {0x32, 0x36}, {0x19, 0x01}, {0x1a, 0x97},
    {0x03, 0x0f}, {0x37, 0x40}, {0x4f, 0xbb}, {0x50, 0x9c},
    {0x5a, 0x57}, {0x6d, 0x80}, {0x3d, 0x34}, {0x39, 0x02},
    {0x35, 0x88}, {0x22, 0x0a}, {0x37, 0x40}, {0x34, 0xa0},
    {0x06, 0x02}, {0x0d, 0xb7}, {0x0e, 0x01}, {0xff, 0x00},
    {0xe0, 0x04}, {0xc0, 0xc8}, {0xc1, 0x96}, {0x86, 0x3d},
    {0x50, 0x89}, {0x51, 0x90}, {0x52, 0x2c}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x88}, {0x57, 0x00}, {0x5a, 0xa0},
    {0x5b, 0x78}, {0x5c, 0x00}, {0xd3, 0x04}, {0xe0, 0x00},
    {0xff, 0xff},
};

const unsigned char OV2640_800x600_JPEG[][2] = {
    {0xFF, 0x01}, {0x11, 0x01}, {0x12, 0x00}, {0x17, 0x11},
    {0x18, 0x75}, {0x32, 0x36}, {0x19, 0x01}, {0x1a, 0x97},
    {0x03, 0x0f}, {0x37, 0x40}, {0x4f, 0xbb}, {0x50, 0x9c},
    {0x5a, 0x57}, {0x6d, 0x80}, {0x3d, 0x34}, {0x39, 0x02},
    {0x35, 0x88}, {0x22, 0x0a}, {0x37, 0x40}, {0x34, 0xa0},
    {0x06, 0x02}, {0x0d, 0xb7}, {0x0e, 0x01}, {0xFF, 0x00},
    {0xe0, 0x04}, {0xc0, 0xc8}, {0xc1, 0x96}, {0x86, 0x35},
    {0x50, 0x89}, {0x51, 0x90}, {0x52, 0x2c}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x88}, {0x57, 0x00}, {0x5a, 0xc8},
    {0x5b, 0x96}, {0x5c, 0x00}, {0xd3, 0x02}, {0xe0, 0x00},
    {0xff, 0xff}};

const unsigned char OV2640_1024x768_JPEG[][2] = {
    {0xFF, 0x01}, {0x11, 0x01}, {0x12, 0x00}, {0x17, 0x11},
    {0x18, 0x75}, {0x32, 0x36}, {0x19, 0x01}, {0x1a, 0x97},
    {0x03, 0x0f}, {0x37, 0x40}, {0x4f, 0xbb}, {0x50, 0x9c},
    {0x5a, 0x57}, {0x6d, 0x80}, {0x3d, 0x34}, {0x39, 0x02},
    {0x35, 0x88}, {0x22, 0x0a}, {0x37, 0x40}, {0x34, 0xa0},
    {0x06, 0x02}, {0x0d, 0xb7}, {0x0e, 0x01}, {0xFF, 0x00},
    {0xc0, 0xC8}, {0xc1, 0x96}, {0x8c, 0x00}, {0x86, 0x3D},
    {0x50, 0x00}, {0x51, 0x90}, {0x52, 0x2C}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x88}, {0x5a, 0x00}, {0x5b, 0xC0},
    {0x5c, 0x01}, {0xd3, 0x02}, {0xff, 0xff}};

const unsigned char OV2640_1280x960_JPEG[][2] = {
    {0xFF, 0x01}, {0x11, 0x01}, {0x12, 0x00}, {0x17, 0x11},
    {0x18, 0x75}, {0x32, 0x36}, {0x19, 0x01}, {0x1a, 0x97},
    {0x03, 0x0f}, {0x37, 0x40}, {0x4f, 0xbb}, {0x50, 0x9c},
    {0x5a, 0x57}, {0x6d, 0x80}, {0x3d, 0x34}, {0x39, 0x02},
    {0x35, 0x88}, {0x22, 0x0a}, {0x37, 0x40}, {0x34, 0xa0},
    {0x06, 0x02}, {0x0d, 0xb7}, {0x0e, 0x01}, {0xFF, 0x00},
    {0xe0, 0x04}, {0xc0, 0xc8}, {0xc1, 0x96}, {0x86, 0x3d},
    {0x50, 0x00}, {0x51, 0x90}, {0x52, 0x2c}, {0x53, 0x00},
    {0x54, 0x00}, {0x55, 0x88}, {0x57, 0x00}, {0x5a, 0x40},
    {0x5b, 0xf0}, {0x5c, 0x01}, {0xd3, 0x02}, {0xe0, 0x00},
    {0xff, 0xff}};

const unsigned char OV2640_CONTRAST2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04},
    {0x7c, 0x07}, {0x7d, 0x20}, {0x7d, 0x28},
    {0x7d, 0x0c}, {0x7d, 0x06}, {0xff, 0xff}};

const unsigned char OV2640_CONTRAST1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04},
    {0x7c, 0x07}, {0x7d, 0x20}, {0x7d, 0x24},
    {0x7d, 0x16}, {0x7d, 0x06}, {0xff, 0xff}};

const unsigned char OV2640_CONTRAST0[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04},
    {0x7c, 0x07}, {0x7d, 0x20}, {0x7d, 0x20},
    {0x7d, 0x20}, {0x7d, 0x06}, {0xff, 0xff}};

const unsigned char OV2640_CONTRAST_1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04},
    {0x7c, 0x07}, {0x7d, 0x20}, {0x7d, 0x1c},
    {0x7d, 0x2a}, {0x7d, 0x06}, {0xff, 0xff}};

const unsigned char OV2640_CONTRAST_2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04},
    {0x7c, 0x07}, {0x7d, 0x20}, {0x7d, 0x18},
    {0x7d, 0x34}, {0x7d, 0x06}, {0xff, 0xff}};

const unsigned char OV2640_SATURATION2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x02}, {0x7c, 0x03},
    {0x7d, 0x68}, {0x7d, 0x68}, {0xff, 0xff}};

const unsigned char OV2640_SATURATION1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x02}, {0x7c, 0x03},
    {0x7d, 0x58}, {0x7d, 0x68}, {0xff, 0xff}};

const unsigned char OV2640_SATURATION0[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x02}, {0x7c, 0x03},
    {0x7d, 0x48}, {0x7d, 0x48}, {0xff, 0xff}};

const unsigned char OV2640_SATURATION_1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x02}, {0x7c, 0x03},
    {0x7d, 0x38}, {0x7d, 0x38}, {0xff, 0xff}};

const unsigned char OV2640_SATURATION_2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x02}, {0x7c, 0x03},
    {0x7d, 0x28}, {0x7d, 0x28}, {0xff, 0xff}};

const unsigned char OV2640_BRIGHTNESS2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04}, {0x7c, 0x09},
    {0x7d, 0x40}, {0x7d, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_BRIGHTNESS1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04}, {0x7c, 0x09},
    {0x7d, 0x30}, {0x7d, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_BRIGHTNESS0[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04}, {0x7c, 0x09},
    {0x7d, 0x20}, {0x7d, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_BRIGHTNESS_1[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04}, {0x7c, 0x09},
    {0x7d, 0x10}, {0x7d, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_BRIGHTNESS_2[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x04}, {0x7c, 0x09},
    {0x7d, 0x00}, {0x7d, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_NORMAL[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x00}, {0x7c, 0x05},
    {0x7d, 0x80}, {0x7d, 0x80}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_ANTIQUE[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x18}, {0x7c, 0x05},
    {0x7d, 0x40}, {0x7d, 0xa6}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_BLACK_NEGATIVE[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x58}, {0x7c, 0x05},
    {0x7d, 0x80}, {0x7d, 0x80}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_BLUISH[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x18}, {0x7c, 0x05},
    {0x7d, 0xa0}, {0x7d, 0x40}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_BLACK[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x18}, {0x7c, 0x05},
    {0x7d, 0x80}, {0x7d, 0x80}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_NEGATIVE[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x40}, {0x7c, 0x05},
    {0x7d, 0x80}, {0x7d, 0x80}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_GREENISH[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x18}, {0x7c, 0x05},
    {0x7d, 0x40}, {0x7d, 0x40}, {0xff, 0xff}};

const unsigned char OV2640_SPECIAL_EFFECTS_REDDISH[][2] = {
    {0xff, 0x00}, {0x7c, 0x00}, {0x7d, 0x18}, {0x7c, 0x05},
    {0x7d, 0x40}, {0x7d, 0xc0}, {0xff, 0xff}};

const unsigned char OV2640_LIGHT_MODE_AUTO[][2] = {
    {0xff, 0x00}, {0xc7, 0x00}, {0xff, 0xff}};

const unsigned char OV2640_LIGHT_MODE_SUNNY[][2] = {
    {0xff, 0x00}, {0xc7, 0x40}, {0xcc, 0x5e},
    {0xcd, 0x41}, {0xce, 0x54}, {0xff, 0xff}};

const unsigned char OV2640_LIGHT_MODE_CLOUDY[][2] = {
    {0xff, 0x00}, {0xc7, 0x40}, {0xcc, 0x65},
    {0xcd, 0x41}, {0xce, 0x4f}, {0xff, 0xff}};

const unsigned char OV2640_LIGHT_MODE_OFFICE[][2] = {
    {0xff, 0x00}, {0xc7, 0x40}, {0xcc, 0x52},
    {0xcd, 0x41}, {0xce, 0x66}, {0xff, 0xff}};

const unsigned char OV2640_LIGHT_MODE_HOME[][2] = {
    {0xff, 0x00}, {0xc7, 0x40}, {0xcc, 0x42},
    {0xcd, 0x3f}, {0xce, 0x71}, {0xff, 0xff}};

/**
 * Camera initialization.
 * @param p_hi2c Pointer to I2C interface.
 * @param p_hdcmi Pointer to DCMI interface.
 */
void OV2640_Init(I2C_HandleTypeDef* p_hi2c,
                 DCMI_HandleTypeDef* p_hdcmi)
{
    phi2c  = p_hi2c;
    phdcmi = p_hdcmi;

    // Hardware reset
    HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin,
                      GPIO_PIN_SET);
    HAL_Delay(100);

    // Software reset: reset all registers to default values
    SCCB_Write(0xff, 0x01);
    SCCB_Write(0x12, 0x80);
    HAL_Delay(100);

#ifdef DEBUG
    uint8_t pid;
    uint8_t ver;
    SCCB_Read(0x0a, &pid); // pid value is 0x26
    SCCB_Read(0x0b, &ver); // ver value is 0x42
    my_printf("PID: 0x%x, VER: 0x%x\n", pid, ver);
#endif

    // Stop DCMI clear buffer
    OV2640_StopDCMI();
}

/**
 * Camera resolution selection.
 * @param opt Resolution option.
 */
void OV2640_ResolutionOptions(uint16_t opt)
{
    switch (opt) {
    case 15533:
        OV2640_ResolutionConfiguration(0);
        break;
    case 15534:
        OV2640_ResolutionConfiguration(1);
        break;
    case 15535:
        OV2640_ResolutionConfiguration(2);
        break;
    case 25535:
        OV2640_ResolutionConfiguration(3);
        break;
    case 45535:
        OV2640_ResolutionConfiguration(4);
        break;
    case 65535:
        OV2640_ResolutionConfiguration(5);
        break;
    default:
        OV2640_ResolutionConfiguration(1);
        break;
    }
}

/**
 * Camera resolution selection.
 * @param opt Resolution option.
 */
void OV2640_ResolutionConfiguration(short opt)
{
#ifdef DEBUG
    my_printf("Starting resolution choice \r\n");
#endif
    OV2640_Configuration(OV2640_JPEG_INIT);
    OV2640_Configuration(OV2640_YUV422);
    OV2640_Configuration(OV2640_JPEG);
    HAL_Delay(10);
    SCCB_Write(0xff, 0x01);
    HAL_Delay(10);
    SCCB_Write(0x15, 0x00);

    switch (opt) {
    case 0:
        OV2640_Configuration(OV2640_160x120_JPEG);
        break;
    case 1:
        OV2640_Configuration(OV2640_320x240_JPEG);
        break;
    case 2:
        OV2640_Configuration(OV2640_640x480_JPEG);
        break;
    case 3:
        OV2640_Configuration(OV2640_800x600_JPEG);
        break;
    case 4:
        OV2640_Configuration(OV2640_1024x768_JPEG);
        break;
    case 5:
        OV2640_Configuration(OV2640_1280x960_JPEG);
        break;
    default:
        OV2640_Configuration(OV2640_320x240_JPEG);
        break;
    }

#ifdef DEBUG
    my_printf("Finalize configuration \r\n");
#endif
}

/**
 * Configure camera registers.
 * @param arr Array with addresses and values using to overwrite
 * camera registers.
 */
void OV2640_Configuration(const unsigned char arr[][2])
{
    unsigned short i = 0;
    uint8_t reg_addr, data, data_read;
    while (1) {
        reg_addr = arr[i][0];
        data     = arr[i][1];
        if (reg_addr == 0xff && data == 0xff) {
            break;
        }
        SCCB_Read(reg_addr, &data_read);
        SCCB_Write(reg_addr, data);
#ifdef DEBUG
        my_printf("SCCB write: 0x%x 0x%x=>0x%x\r\n", reg_addr,
                  data_read, data);
#endif
        HAL_Delay(10);
        SCCB_Read(reg_addr, &data_read);
        if (data != data_read) {
#ifdef DEBUG
            my_printf("SCCB write failure: 0x%x 0x%x\r\n", reg_addr,
                      data_read);
#endif
        }
        i++;
    }
}

/**
 *  Changing the special effect applied to a photo.
 * @param specialEffect Name or value of the special effect.
 */
void OV2640_SpecialEffect(short specialEffect)
{
#ifdef DEBUG
    my_printf("Special effect value:%d\r\n", specialEffect);
#endif
    if (specialEffect == 0) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_ANTIQUE);
    } else if (specialEffect == 1) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_BLUISH);
    } else if (specialEffect == 2) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_GREENISH);
    } else if (specialEffect == 3) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_REDDISH);
    } else if (specialEffect == 4) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_BLACK);
    } else if (specialEffect == 5) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_NEGATIVE);
    } else if (specialEffect == 6) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_BLACK_NEGATIVE);
    } else if (specialEffect == 7) {
        OV2640_Configuration(OV2640_SPECIAL_EFFECTS_NORMAL);
    }
}

/**
 * Activation of simple white balance.
 */
void OV2640_AdvancedWhiteBalance()
{
#ifdef DEBUG
    my_printf("Enable simple white balance mode\r\n");
#endif
    SCCB_Write(0xff, 0x00);
    HAL_Delay(1);
    SCCB_Write(0xc7, 0x00);
}

/**
 * Activation of simple white balance.
 */
void OV2640_SimpleWhiteBalance()
{
#ifdef DEBUG
    my_printf("Enable simple white balance mode\r\n");
#endif
    SCCB_Write(0xff, 0x00);
    HAL_Delay(1);
    SCCB_Write(0xc7, 0x10);
}

/**
 * Changing image brightness.
 * @param brightness Name or value of the brightness.
 */
void OV2640_Brightness(short brightness)
{
#ifdef DEBUG
    my_printf("Brightness value:%d\r\n", brightness);
#endif

    if (brightness == 0) {
        OV2640_Configuration(OV2640_BRIGHTNESS0);
    } else if (brightness == 1) {
        OV2640_Configuration(OV2640_BRIGHTNESS1);
    } else if (brightness == 2) {
        OV2640_Configuration(OV2640_BRIGHTNESS2);
    } else if (brightness == 3) {
        OV2640_Configuration(OV2640_BRIGHTNESS_1);
    } else if (brightness == 4) {
        OV2640_Configuration(OV2640_BRIGHTNESS_2);
    }
}

/**
 * Changing image light mode.
 * @param lightMode Name or value of the light mode.
 */
void OV2640_LightMode(short lightMode)
{
#ifdef DEBUG
    my_printf("Light mode value:%d\r\n", lightMode);
#endif

    if (lightMode == 0) {
        OV2640_AdvancedWhiteBalance();
    } else if (lightMode == 1) {
        OV2640_Configuration(OV2640_LIGHT_MODE_SUNNY);
    } else if (lightMode == 2) {
        OV2640_Configuration(OV2640_LIGHT_MODE_CLOUDY);
    } else if (lightMode == 3) {
        OV2640_Configuration(OV2640_LIGHT_MODE_OFFICE);
    } else if (lightMode == 4) {
        OV2640_Configuration(OV2640_LIGHT_MODE_HOME);
    }
}
/**
 *  Changing image saturation.
 * @param saturation  Name or value of the saturation.
 */
void OV2640_Saturation(short saturation)
{
#ifdef DEBUG
    my_printf("Saturation value:%d\r\n", saturation);
#endif

    if (saturation == 0) {
        OV2640_Configuration(OV2640_SATURATION0);
    } else if (saturation == 1) {
        OV2640_Configuration(OV2640_SATURATION1);
    } else if (saturation == 2) {
        OV2640_Configuration(OV2640_SATURATION2);
    } else if (saturation == 3) {
        OV2640_Configuration(OV2640_SATURATION_1);
    } else if (saturation == 4) {
        OV2640_Configuration(OV2640_SATURATION_2);
    }
}

/**
 * Changing image contrast.
 * @param contrast Name or value of the contrast.
 */
void OV2640_Contrast(short contrast)
{
#ifdef DEBUG
    my_printf("Contrast value:%d\r\n", contrast);
#endif

    if (contrast == 0) {
        OV2640_Configuration(OV2640_CONTRAST0);
    } else if (contrast == 1) {
        OV2640_Configuration(OV2640_CONTRAST1);
    } else if (contrast == 2) {
        OV2640_Configuration(OV2640_CONTRAST2);
    } else if (contrast == 3) {
        OV2640_Configuration(OV2640_CONTRAST_1);
    } else if (contrast == 4) {
        OV2640_Configuration(OV2640_CONTRAST_2);
    }
}

/**
 * Stop DCMI (Clear  memory buffer)
 */
void OV2640_StopDCMI(void)
{
#ifdef DEBUG
    my_printf("DCMI has been stopped \r\n");
#endif
    HAL_DCMI_Stop(phdcmi);
    HAL_Delay(10); // If you get a DCMI error (data is not received),
                   // increase value to 30.
}

/**
 * Executes a single reading from DCMI and returns  data as an image.
 * @param frameBuffer Table with data.
 * @param length Length of capture to be transferred.
 */
void OV2640_CaptureSnapshot(uint32_t frameBuffer, int length)
{
    HAL_DCMI_Start_DMA(phdcmi, DCMI_MODE_SNAPSHOT, frameBuffer,
                       length);
    HAL_Delay(2000);
    HAL_DCMI_Suspend(phdcmi);
    HAL_DCMI_Stop(phdcmi);
}

/**
 * Write value to camera register.
 * @param reg_addr Address of register.
 * @param data New value.
 * @return  Operation status.
 */
short SCCB_Write(uint8_t reg_addr, uint8_t data)
{
    short opertionStatus = 0;
    uint8_t buffer[2]    = {0};
    HAL_StatusTypeDef connectionStatus;
    buffer[0] = reg_addr;
    buffer[1] = data;
    __disable_irq();
    connectionStatus = HAL_I2C_Master_Transmit(phi2c, (uint16_t)0x60,
                                               buffer, 2, 100);
    if (connectionStatus == HAL_OK) {
        opertionStatus = 1;
    } else {
        opertionStatus = 0;
    }
    __enable_irq();
    return opertionStatus;
}

/**
 * Reading data from camera registers.
 * @param reg_addr Address of register.
 * @param pdata Value read from register.
 * @return Operation status.
 */
short SCCB_Read(uint8_t reg_addr, uint8_t* pdata)
{
    short opertionStatus = 0;
    HAL_StatusTypeDef connectionStatus;
    __disable_irq();
    connectionStatus = HAL_I2C_Master_Transmit(phi2c, (uint16_t)0x60,
                                               &reg_addr, 1, 100);
    if (connectionStatus == HAL_OK) {
        connectionStatus = HAL_I2C_Master_Receive(
            phi2c, (uint16_t)0x61, pdata, 1, 100);
        if (connectionStatus == HAL_OK) {
            opertionStatus = 0;
        } else {
            opertionStatus = 1;
        }
    } else {
        opertionStatus = 2;
    }
    __enable_irq();
    return opertionStatus;
}
