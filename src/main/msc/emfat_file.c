/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: jflyper@github.com
 */

#include "common/utils.h"
#include "common/printf.h"

#include "emfat.h"
#include "emfat_file.h"

#include "io/flashfs.h"

#define FILESYSTEM_SIZE_MB 256

#define USE_EMFAT_AUTORUN
#define USE_EMFAT_ICON
//#define USE_EMFAT_README

#ifdef USE_EMFAT_AUTORUN
static const char autorun_file[] =
    "[autorun]\r\n"
    "icon=icon.ico\r\n"
    "label=Betaflight Onboard Flash\r\n" ;
#define AUTORUN_SIZE (sizeof(autorun_file) - 1)
#define EMFAT_INCR_AUTORUN 1
#else
#define EMFAT_INCR_AUTORUN 0
#endif

#ifdef USE_EMFAT_README
static const char readme_file[] =
    "This is readme file\r\n";
#define README_SIZE  (sizeof(readme_file) - 1)
#define EMFAT_INCR_README 1
#else
#define EMFAT_INCR_README 0
#endif

#ifdef USE_EMFAT_ICON
static const char icon_file[] =
{
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x18, 0x18, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x28, 0x09,
    0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x30, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xfc,
    0xfc, 0xde, 0xfc, 0xfc, 0xfc, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfb, 0xfb,
    0xfb, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfc, 0xfc,
    0xfc, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xde, 0xfb, 0xfb,
    0xfb, 0xf4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xf4, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd,
    0xfd, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfc, 0xfc,
    0xfc, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xfb,
    0xfc, 0xff, 0xd7, 0xdd, 0xe1, 0xff, 0xc7, 0xc3, 0xc2, 0xff, 0xce, 0xce, 0xce, 0xff, 0xe2, 0xe4,
    0xe5, 0xff, 0xfd, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfb, 0xfb,
    0xfb, 0xee, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb0, 0xad, 0xad, 0xff, 0x3a, 0x89,
    0xa8, 0xff, 0x03, 0xb5, 0xed, 0xff, 0x20, 0x57, 0x6c, 0xff, 0x36, 0x25, 0x1f, 0xff, 0x34, 0x52,
    0x5e, 0xff, 0x4f, 0x65, 0x6e, 0xff, 0x8a, 0x86, 0x84, 0xff, 0xd8, 0xd8, 0xd8, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xee, 0xfa, 0xfa,
    0xfa, 0xee, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfc, 0xfc,
    0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb7, 0xb2, 0xb1, 0xff, 0x2a, 0x1d, 0x18, 0xff, 0x33, 0x2a,
    0x26, 0xff, 0x25, 0x68, 0x7f, 0xff, 0x16, 0x90, 0xbe, 0xff, 0x3a, 0x38, 0x37, 0xff, 0x37, 0x38,
    0x38, 0xff, 0x31, 0x35, 0x37, 0xff, 0x29, 0x28, 0x27, 0xff, 0x34, 0x34, 0x34, 0xff, 0x93, 0x93,
    0x93, 0xff, 0xf3, 0xf3, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0xf8, 0xff, 0x4f, 0xa0, 0xbf, 0xff, 0x0d, 0x8c, 0xb8, 0xff, 0x2c, 0x62,
    0x76, 0xff, 0x3e, 0x31, 0x2e, 0xff, 0x36, 0x46, 0x4d, 0xff, 0x38, 0x41, 0x44, 0xff, 0x3b, 0x39,
    0x38, 0xff, 0x37, 0x36, 0x35, 0xff, 0x2c, 0x2c, 0x2c, 0xff, 0x28, 0x28, 0x28, 0xff, 0x83, 0x83,
    0x83, 0xff, 0x77, 0x77, 0x77, 0xff, 0xd5, 0xd5, 0xd5, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfc, 0xfc,
    0xfc, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfd, 0xfb, 0xff, 0x5f, 0x98, 0xad, 0xff, 0x1a, 0x5c, 0x73, 0xff, 0x30, 0x5a,
    0x6a, 0xff, 0x38, 0x42, 0x46, 0xff, 0x3c, 0x35, 0x33, 0xff, 0x39, 0x38, 0x37, 0xff, 0x2b, 0x2b,
    0x2b, 0xff, 0x3c, 0x3c, 0x3c, 0xff, 0x83, 0x83, 0x83, 0xff, 0xc2, 0xc2, 0xc2, 0xff, 0xd2, 0xd2,
    0xd2, 0xff, 0xa9, 0xa9, 0xa9, 0xff, 0x86, 0x86, 0x86, 0xff, 0xdf, 0xdf, 0xdf, 0xee, 0xfc, 0xfc,
    0xfc, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xba, 0xb7, 0xff, 0x2c, 0x21, 0x1d, 0xff, 0x38, 0x30,
    0x2d, 0xff, 0x3c, 0x3a, 0x39, 0xff, 0x34, 0x35, 0x35, 0xff, 0x2e, 0x2e, 0x2e, 0xff, 0x78, 0x78,
    0x78, 0xff, 0xdf, 0xdf, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xeb, 0xeb, 0xeb, 0xee, 0xfa, 0xfa,
    0xfa, 0xee, 0xfe, 0xfe, 0xfe, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfc, 0xfc,
    0xfc, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb3, 0xb0, 0xaf, 0xff, 0x3d, 0x31,
    0x2c, 0xff, 0x33, 0x30, 0x2f, 0xff, 0x49, 0x4a, 0x4a, 0xff, 0xcb, 0xcb, 0xcb, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc,
    0xfc, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd3, 0xe2, 0xe8, 0xff, 0x1c, 0x85,
    0xae, 0xff, 0x2f, 0x44, 0x4c, 0xff, 0x41, 0x3c, 0x3a, 0xff, 0x9a, 0x9a, 0x9a, 0xff, 0xf9, 0xf9,
    0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xf8, 0xf8, 0xf8, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf8, 0xf8, 0xff, 0xf5, 0xf5,
    0xf5, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8a, 0xd0, 0xea, 0xff, 0x00, 0xb0,
    0xf7, 0xff, 0x36, 0x49, 0x51, 0xff, 0x39, 0x33, 0x31, 0xff, 0x1f, 0x1f, 0x1f, 0xff, 0x98, 0x98,
    0x98, 0xff, 0xe2, 0xe2, 0xe2, 0xff, 0x9b, 0x9b, 0x9b, 0xff, 0x71, 0x71, 0x71, 0xff, 0x72, 0x72,
    0x72, 0xff, 0x63, 0x63, 0x63, 0xff, 0xef, 0xef, 0xef, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf4, 0xf4,
    0xf4, 0xff, 0xae, 0xae, 0xae, 0xff, 0x8c, 0x8c, 0x8c, 0xff, 0x86, 0x86, 0x86, 0xff, 0x84, 0x84,
    0x84, 0xff, 0x86, 0x86, 0x86, 0xff, 0x91, 0x8a, 0x87, 0xff, 0x48, 0x7a, 0x8d, 0xff, 0x00, 0x9c,
    0xd7, 0xff, 0x3a, 0x3e, 0x3f, 0xff, 0x39, 0x36, 0x35, 0xff, 0x35, 0x35, 0x35, 0xff, 0x42, 0x42,
    0x42, 0xff, 0x3b, 0x3b, 0x3b, 0xff, 0x31, 0x31, 0x31, 0xff, 0x92, 0x92, 0x92, 0xff, 0x90, 0x90,
    0x90, 0xff, 0x7b, 0x7b, 0x7b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfb, 0xfb,
    0xfb, 0xee, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xde, 0xde, 0xde, 0xff, 0x80, 0x80,
    0x80, 0xff, 0xb1, 0xb1, 0xb1, 0xff, 0xe6, 0xe6, 0xe6, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xea, 0xea, 0xea, 0xff, 0x81, 0x80, 0x80, 0xff, 0x49, 0x42, 0x40, 0xff, 0x55, 0x8e,
    0xa3, 0xff, 0x1f, 0x5b, 0x72, 0xff, 0x35, 0x3a, 0x3c, 0xff, 0x43, 0x41, 0x41, 0xff, 0x35, 0x35,
    0x35, 0xff, 0x2a, 0x2a, 0x2a, 0xff, 0xbe, 0xbe, 0xbe, 0xff, 0xe8, 0xe8, 0xe8, 0xff, 0x6f, 0x6f,
    0x6f, 0xff, 0xe7, 0xe7, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xee, 0xfb, 0xfb,
    0xfb, 0xee, 0xff, 0xff, 0xff, 0xff, 0xf5, 0xf5, 0xf5, 0xff, 0x7b, 0x7b, 0x7b, 0xff, 0xe0, 0xe0,
    0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf8, 0xf8, 0xff, 0xb8, 0xb8,
    0xb8, 0xff, 0x7d, 0x7d, 0x7d, 0xff, 0xa3, 0xa3, 0xa3, 0xff, 0xd2, 0xd2, 0xd2, 0xff, 0x87, 0x7c,
    0x77, 0xff, 0x5a, 0x57, 0x55, 0xff, 0xca, 0xcb, 0xcb, 0xff, 0x82, 0x82, 0x82, 0xff, 0x23, 0x23,
    0x23, 0xff, 0x38, 0x38, 0x38, 0xff, 0x43, 0x43, 0x43, 0xff, 0x5d, 0x5d, 0x5d, 0xff, 0xd9, 0xd9,
    0xd9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xff, 0xff, 0xff, 0xff, 0xa1, 0xa1, 0xa1, 0xff, 0xb6, 0xb6, 0xb6, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xdb, 0xdb, 0xdb, 0xff, 0xb1, 0xb1, 0xb1, 0xff, 0xc7, 0xc7,
    0xc7, 0xff, 0xea, 0xea, 0xea, 0xff, 0xa6, 0xa6, 0xa6, 0xff, 0x60, 0x60, 0x60, 0xff, 0x9f, 0x9f,
    0x9f, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd1, 0xd1, 0xd1, 0xff, 0x4e, 0x4e,
    0x4e, 0xff, 0x29, 0x29, 0x29, 0xff, 0x73, 0x73, 0x73, 0xff, 0xeb, 0xeb, 0xeb, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xec, 0xec, 0xec, 0xff, 0x90, 0x90, 0x90, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xf3, 0xf3, 0xf3, 0xff, 0xf0, 0xf0, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb7, 0xb7,
    0xb7, 0xff, 0x61, 0x61, 0x61, 0xff, 0x89, 0x89, 0x89, 0xff, 0xeb, 0xeb, 0xeb, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xdc, 0xdc, 0xdc, 0xff, 0x99, 0x99,
    0x99, 0xff, 0xbc, 0xbc, 0xbc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0x98, 0x98, 0x98, 0xff, 0x9c, 0x9c, 0x9c, 0xff, 0xf9, 0xf9, 0xf9, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc5, 0xc5, 0xc5, 0xff, 0x66, 0x66, 0x66, 0xff, 0x76, 0x76,
    0x76, 0xff, 0xdb, 0xdb, 0xdb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xff, 0xdd, 0xdd, 0xdd, 0xff, 0xcd, 0xcd, 0xcd, 0xff, 0xf7, 0xf7,
    0xf7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xee, 0xda, 0xda,
    0xda, 0xee, 0x48, 0x48, 0x48, 0xff, 0x75, 0x75, 0x75, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd5, 0xd5,
    0xd5, 0xff, 0x72, 0x72, 0x72, 0xff, 0x6a, 0x6a, 0x6a, 0xff, 0xc8, 0xc8, 0xc8, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xee, 0xeb, 0xeb,
    0xeb, 0xee, 0xaf, 0xaf, 0xaf, 0xff, 0xb0, 0xb0, 0xb0, 0xff, 0x8b, 0x8b, 0x8b, 0xff, 0x60, 0x60,
    0x60, 0xff, 0xb3, 0xb3, 0xb3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe,
    0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0x66, 0x66, 0x66, 0xff, 0x54, 0x54, 0x54, 0xff, 0xa0, 0xa0, 0xa0, 0xff, 0xfa, 0xfa,
    0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfd, 0xfd,
    0xfd, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xfd, 0xfd,
    0xfd, 0xff, 0xfc, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xce, 0xce,
    0xce, 0xee, 0x98, 0x98, 0x98, 0xff, 0xef, 0xef, 0xef, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xee, 0xf6, 0xf6,
    0xf6, 0xf4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfb, 0xfb, 0xf4, 0xfd, 0xfd,
    0xfd, 0xde, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfc, 0xfc, 0xfc, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfb, 0xfb, 0xfb, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd,
    0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xee, 0xfd, 0xfd, 0xfd, 0xde,
};

#define ICON_SIZE    (sizeof(icon_file))
#define EMFAT_INCR_ICON 1
#else
#define EMFAT_INCR_ICON 0
#endif

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1,1,2018, 13,0,0)
#define CMA { CMA_TIME, CMA_TIME, CMA_TIME }

static void memory_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    int len;

    if (offset > entry->curr_size) {
        return;
    }

    if (offset + size > entry->curr_size) {
        len = entry->curr_size - offset;
    } else {
        len = size;
    }

    memcpy(dest, &((char *)entry->user_data)[offset], len);
}

static void bblog_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    UNUSED(entry);

    flashfsReadAbs(offset, dest, size);
}

static const emfat_entry_t entriesPredefined[] =
{
    // name           dir    attr         lvl offset  size             max_size        user                time  read               write
    { "",             true,  0,           0,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
#ifdef USE_EMFAT_AUTORUN
    { "autorun.inf",  false, ATTR_HIDDEN, 1,  0,      AUTORUN_SIZE,    AUTORUN_SIZE,   (long)autorun_file, CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_ICON
    { "icon.ico",     false, ATTR_HIDDEN, 1,  0,      ICON_SIZE,       ICON_SIZE,      (long)icon_file,    CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_README
    { "readme.txt",   false, 0,           1,  0,      README_SIZE,     1024*1024,      (long)readme_file,  CMA,  memory_read_proc,  NULL, { 0 } },
#endif
    { "BTTR_ALL.BBL", 0,     0,           1,  0,      0,               0,              0,                  CMA,  bblog_read_proc,   NULL, { 0 } },
    { "PADDING.TXT",  0,     ATTR_HIDDEN, 1,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
};

#define PREDEFINED_ENTRY_COUNT (1 + EMFAT_INCR_AUTORUN + EMFAT_INCR_ICON + EMFAT_INCR_README)
#define APPENDED_ENTRY_COUNT 2

#define EMFAT_MAX_LOG_ENTRY 100
#define EMFAT_MAX_ENTRY (PREDEFINED_ENTRY_COUNT + EMFAT_MAX_LOG_ENTRY + APPENDED_ENTRY_COUNT)

static emfat_entry_t entries[EMFAT_MAX_ENTRY];
static char logNames[EMFAT_MAX_LOG_ENTRY][8+3];

emfat_t emfat;

static void emfat_add_log(emfat_entry_t *entry, int number, uint32_t offset, uint32_t size)
{
    tfp_sprintf(logNames[number], "BTTR_%03d.BBL", number + 1);
    entry->name = logNames[number];
    entry->level = 1;
    entry->offset = offset;
    entry->curr_size = size;
    entry->max_size = entry->curr_size;
    entry->cma_time[0] = CMA_TIME;
    entry->cma_time[1] = CMA_TIME;
    entry->cma_time[2] = CMA_TIME;
    entry->readcb = bblog_read_proc;
}

static int emfat_find_log(emfat_entry_t *entry, int maxCount)
{
    uint32_t limit  = flashfsIdentifyStartOfFreeSpace();
    uint32_t lastOffset = 0;
    uint32_t currOffset = 0;
    int fileNumber = 0;
    uint8_t buffer[18];
    int logCount = 0;

    for ( ; currOffset < limit ; currOffset += 2048) { // XXX 2048 = FREE_BLOCK_SIZE in io/flashfs.c

        flashfsReadAbs(currOffset, buffer, 18);

        if (strncmp((char *)buffer, "H Product:Blackbox", 18)) {
            continue;
        }

        if (lastOffset != currOffset) {
            emfat_add_log(entry, fileNumber, lastOffset, currOffset - lastOffset);

            ++fileNumber;
            ++logCount;
            if (fileNumber == maxCount) {
                break;
            }
            ++entry;
        }

        lastOffset = currOffset;
    }

    if (fileNumber != maxCount && lastOffset != currOffset) {
        emfat_add_log(entry, fileNumber, lastOffset, currOffset - lastOffset);
        ++logCount;
    }
    return logCount;
}

void emfat_init_files(void)
{
    emfat_entry_t *entry;
    memset(entries, 0, sizeof(entries));

    for (size_t i = 0 ; i < PREDEFINED_ENTRY_COUNT ; i++) {
        entries[i] = entriesPredefined[i];
    }

    // Detect and create entries for each individual log
    const int logCount = emfat_find_log(&entries[PREDEFINED_ENTRY_COUNT], EMFAT_MAX_LOG_ENTRY);

    int entryIndex = PREDEFINED_ENTRY_COUNT + logCount;

    if (logCount > 0) {
        // Create the all logs entry that represents all used flash space to
        // allow downloading the entire log in one file
        entries[entryIndex] = entriesPredefined[PREDEFINED_ENTRY_COUNT];
        entry = &entries[entryIndex];
        entry->curr_size = flashfsIdentifyStartOfFreeSpace();
        entry->max_size = entry->curr_size;
        ++entryIndex;
    }

    // Padding file to fill out the filesystem size to FILESYSTEM_SIZE_MB
    entries[entryIndex] = entriesPredefined[PREDEFINED_ENTRY_COUNT + 1];
    entry = &entries[entryIndex];
    // used space is doubled because of the individual files plus the single complete file
    entry->curr_size = (FILESYSTEM_SIZE_MB * 1024 * 1024) - (flashfsIdentifyStartOfFreeSpace() * 2);
    entry->max_size = entry->curr_size;

    emfat_init(&emfat, "BUTTERF", entries);
}
