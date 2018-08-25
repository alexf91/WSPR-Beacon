/*
 * si5351-avr-tiny-minimal.c - Minimal Si5351 library for avr-gcc and
 * ATtiny microprocessors with 8 kB of flash memory and a USI module.
 *
 * Copyright (C) 2015 Jason Milldrum <milldrum@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "i2c_master.h"
#include "si5351.h"

uint32_t xtal_freq = SI5351_XTAL_FREQ;
int32_t ref_correction;

struct Si5351Status dev_status;
struct Si5351IntStatus dev_int_status;

/*
 * si5351_init(uint8_t xtal_load_c, uint32_t ref_osc_freq)
 *
 * Setup communications to the Si5351 and set the crystal
 * load capacitance.
 *
 * xtal_load_c - Crystal load capacitance. Use the SI5351_CRYSTAL_LOAD_*PF
 * defines in the header file
 * ref_osc_freq - Crystal/reference oscillator frequency in 1 Hz increments.
 * Defaults to 25000000 if a 0 is used here.
 *
 */
void si5351_init(uint8_t xtal_load_c, uint32_t ref_osc_freq) {
    //i2c_init();

    /* Set crystal load capacitance */
    si5351_write(SI5351_CRYSTAL_LOAD, xtal_load_c);

    // Change the ref osc freq if different from default
    if (ref_osc_freq != 0) {
        xtal_freq = ref_osc_freq;
    }

    // Initialize the CLK outputs according to flowchart in datasheet
    // First, turn them off
    si5351_write(16, 0x80);
    si5351_write(17, 0x80);
    si5351_write(18, 0x80);

    // Turn the clocks back on...
    si5351_write(16, 0x0c);
    si5351_write(17, 0x0c);
    si5351_write(18, 0x0c);

    // Then reset the PLLs
    si5351_pll_reset(SI5351_PLLA);
    si5351_pll_reset(SI5351_PLLB);
}

/*
 * si5351_set_freq(uint64_t freq, enum si5351_clock clk)
 *
 * Uses SI5351_PLL_FIXED (900 MHz) for PLLA.
 * All multisynths are assigned to PLLA using this function.
 * PLLA is set to 900 MHz.
 * Restricted to outputs from 1 to 150 MHz.
 * If you need frequencies outside that range, use set_pll()
 * and set_ms() to set the synth dividers manually.
 *
 * freq - Output frequency in Hz
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
void si5351_set_freq(uint64_t freq, enum si5351_clock clk) {
    struct Si5351Frac pll_frac, ms_frac;

    // Lower bounds check
    if (freq < SI5351_MULTISYNTH_MIN_FREQ) {
        freq = SI5351_MULTISYNTH_MIN_FREQ;
    }

    // Upper bounds check
    if (freq > SI5351_MULTISYNTH_DIVBY4_FREQ) {
        freq = SI5351_MULTISYNTH_DIVBY4_FREQ;
    }

    // Set the PLL
    pll_frac.a = (uint16_t)(SI5351_PLL_FIXED / xtal_freq);
    if (ref_correction < 0) {
        pll_frac.b =
            (uint32_t)((pll_frac.a * (uint32_t)(ref_correction * -1)) / 10);
    } else {
        pll_frac.b = 1000000UL -
                     (uint32_t)((pll_frac.a * (uint32_t)(ref_correction)) / 10);
        pll_frac.a--;
    }
    pll_frac.c = 1000000UL;
    si5351_set_pll(pll_frac, SI5351_PLLA);

    // Set the MS
    ms_frac.a = (uint16_t)(SI5351_PLL_FIXED / freq);
    ms_frac.b = (uint32_t)(((SI5351_PLL_FIXED % freq) * 1000000UL) / freq);
    ms_frac.c = 1000000UL;
    si5351_set_ms(clk, ms_frac, 0, SI5351_OUTPUT_CLK_DIV_1, 0);
}

/*
 * si5351_set_pll(struct Si5351Frac frac, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency by
 * using the Si5351Frac struct to specify the synth divider ratio.
 *
 * frac - PLL fractional divider values
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void si5351_set_pll(struct Si5351Frac frac, enum si5351_pll target_pll) {
    struct Si5351RegSet pll_reg;

    // Calculate parameters
    pll_reg.p1 = 128 * frac.a + ((128 * frac.b) / frac.c) - 512;
    pll_reg.p2 = 128 * frac.b - frac.c * ((128 * frac.b) / frac.c);
    pll_reg.p3 = frac.c;

    // Derive the register values to write
    // Prepare an array for parameters to be written to
    uint8_t params[20];
    uint8_t i = 0;
    uint8_t temp;

    // Registers 26-27 for PLLA
    temp = ((pll_reg.p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(pll_reg.p3 & 0xFF);
    params[i++] = temp;

    // Register 28 for PLLA
    temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
    params[i++] = temp;

    // Registers 29-30 for PLLA
    temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(pll_reg.p1 & 0xFF);
    params[i++] = temp;

    // Register 31 for PLLA
    temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
    temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
    params[i++] = temp;

    // Registers 32-33 for PLLA
    temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(pll_reg.p2 & 0xFF);
    params[i++] = temp;

    // Write the parameters
    if (target_pll == SI5351_PLLA) {
        si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, params);
    } else if (target_pll == SI5351_PLLB) {
        si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);
    }
}

/*
 * si5351_set_ms(enum si5351_clock clk, struct Si5351Frac frac, uint8_t
 * int_mode, uint8_t r_div, uint8_t div_by_4)
 *
 * Set the specified multisynth parameters.
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * frac - Synth fractional divider values
 * int_mode - Set integer mode
 *  Set to 1 to enable, 0 to disable
 * r_div - Desired r_div ratio
 * div_by_4 - Set Divide By 4 mode
 *   Set to 1 to enable, 0 to disable
 */
void si5351_set_ms(enum si5351_clock clk, struct Si5351Frac frac,
                   uint8_t int_mode, uint8_t r_div, uint8_t div_by_4) {
    struct Si5351RegSet ms_reg;
    uint8_t params[20];
    uint8_t i = 0;
    uint8_t temp;
    uint8_t reg_val;

    // Calculate parameters
    if (div_by_4 == 1) {
        ms_reg.p3 = 1;
        ms_reg.p2 = 0;
        ms_reg.p1 = 0;
    } else {
        ms_reg.p1 = 128 * frac.a + ((128 * frac.b) / frac.c) - 512;
        ms_reg.p2 = 128 * frac.b - frac.c * ((128 * frac.b) / frac.c);
        ms_reg.p3 = frac.c;
    }

    // Registers 42-43 for CLK0
    temp = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p3 & 0xFF);
    params[i++] = temp;

    // Register 44 for CLK0
    si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8), &reg_val);
    reg_val &= ~(0x03);
    temp = reg_val | ((uint8_t)((ms_reg.p1 >> 16) & 0x03));
    params[i++] = temp;

    // Registers 45-46 for CLK0
    temp = (uint8_t)((ms_reg.p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p1 & 0xFF);
    params[i++] = temp;

    // Register 47 for CLK0
    temp = (uint8_t)((ms_reg.p3 >> 12) & 0xF0);
    temp += (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
    params[i++] = temp;

    // Registers 48-49 for CLK0
    temp = (uint8_t)((ms_reg.p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t)(ms_reg.p2 & 0xFF);
    params[i++] = temp;

    // Write the parameters
    switch (clk) {
        case SI5351_CLK0:
            si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, params);
            break;
        case SI5351_CLK1:
            si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, params);
            break;
        case SI5351_CLK2:
            si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, params);
            break;
        case SI5351_CLK3:
            si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, params);
            break;
        case SI5351_CLK4:
            si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, params);
            break;
        case SI5351_CLK5:
            si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, params);
            break;
        case SI5351_CLK6:
            si5351_write_bulk(SI5351_CLK6_PARAMETERS, i, params);
            break;
        case SI5351_CLK7:
            si5351_write_bulk(SI5351_CLK7_PARAMETERS, i, params);
            break;
    }

    si5351_set_int(clk, int_mode);
    si5351_set_ms_div(clk, r_div, div_by_4);
}

/*
 * si5351_output_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen clock
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void si5351_output_enable(enum si5351_clock clk, uint8_t enable) {
    uint8_t reg_val;

    if (si5351_read(SI5351_OUTPUT_ENABLE_CTRL, &reg_val) != 0) {
        return;
    }

    if (enable == 1) {
        reg_val &= ~(1 << (uint8_t)clk);
    } else {
        reg_val |= (1 << (uint8_t)clk);
    }

    si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

/*
 * si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive) {
    uint8_t reg_val;
    const uint8_t mask = 0x03;

    if (si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val) != 0) {
        return;
    }

    switch (drive) {
        case SI5351_DRIVE_2MA:
            reg_val &= ~(mask);
            reg_val |= 0x00;
            break;
        case SI5351_DRIVE_4MA:
            reg_val &= ~(mask);
            reg_val |= 0x01;
            break;
        case SI5351_DRIVE_6MA:
            reg_val &= ~(mask);
            reg_val |= 0x02;
            break;
        case SI5351_DRIVE_8MA:
            reg_val &= ~(mask);
            reg_val |= 0x03;
            break;
        default:
            break;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_update_status(void)
 *
 * Call this to update the status structs, then access them
 * via the dev_status and dev_int_status global variables.
 *
 * See the header file for the struct definitions. These
 * correspond to the flag names for registers 0 and 1 in
 * the Si5351 datasheet.
 */
void si5351_update_status(void) {
    si5351_update_sys_status(&dev_status);
    si5351_update_int_status(&dev_int_status);
}

/*
 * si5351_set_correction(int32_t corr)
 *
 * Use this to set the oscillator correction factor to
 * EEPROM. This value is a signed 32-bit integer of the
 * parts-per-10 million value that the actual oscillation
 * frequency deviates from the specified frequency.
 *
 * The frequency calibration is done as a one-time procedure.
 * Any desired test frequency within the normal range of the
 * Si5351 should be set, then the actual output frequency
 * should be measured as accurately as possible. The
 * difference between the measured and specified frequencies
 * should be calculated in Hertz, then multiplied by 10 in
 * order to get the parts-per-10 million value.
 *
 * Since the Si5351 itself has an intrinsic 0 PPM error, this
 * correction factor is good across the entire tuning range of
 * the Si5351. Once this calibration is done accurately, it
 * should not have to be done again for the same Si5351 and
 * crystal.
 */
void si5351_set_correction(int32_t corr) {
    // xtal_freq = (uint32_t)(xtal_freq + (corr * (xtal_freq / 10000000UL)));
    ref_correction = corr;
}

/*
 * si5351_set_phase(enum si5351_clock clk, uint8_t phase)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * phase - 7-bit phase word
 *   (in units of VCO/4 period)
 *
 * Write the 7-bit phase register. This must be used
 * with a user-set PLL frequency so that the user can
 * calculate the proper tuning word based on the PLL period.
 */
void si5351_set_phase(enum si5351_clock clk, uint8_t phase) {
    // Mask off the upper bit since it is reserved
    phase = phase & 0b01111111;

    si5351_write(SI5351_CLK0_PHASE_OFFSET + (uint8_t)clk, phase);
}

/*
 * si5351_pll_reset(enum si5351_pll target_pll)
 *
 * target_pll - Which PLL to reset
 *     (use the si5351_pll enum)
 *
 * Apply a reset to the indicated PLL.
 */
void si5351_pll_reset(enum si5351_pll target_pll) {
    if (target_pll == SI5351_PLLA) {
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
    } else if (target_pll == SI5351_PLLB) {
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
    }
}

/*
 * si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pll - Which PLL to use as the source
 *     (use the si5351_pll enum)
 *
 * Set the desired PLL source for a multisynth.
 */
void si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll) {
    uint8_t reg_val;

    si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val);

    if (pll == SI5351_PLLA) {
        reg_val &= ~(SI5351_CLK_PLL_SELECT);
    } else if (pll == SI5351_PLLB) {
        reg_val |= SI5351_CLK_PLL_SELECT;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_set_int(enum si5351_clock clk, uint8_t int_mode)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Set the indicated multisynth into integer mode.
 */
void si5351_set_int(enum si5351_clock clk, uint8_t enable) {
    uint8_t reg_val;
    si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val);

    if (enable == 1) {
        reg_val |= (SI5351_CLK_INTEGER_MODE);
    } else {
        reg_val &= ~(SI5351_CLK_INTEGER_MODE);
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_set_clock_pwr(enum si5351_clock clk, uint8_t pwr)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pwr - Set to 1 to enable, 0 to disable
 *
 * Enable or disable power to a clock output (a power
 * saving feature).
 */
void si5351_set_clock_pwr(enum si5351_clock clk, uint8_t pwr) {
    uint8_t reg_val;
    si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val);

    if (pwr == 1) {
        reg_val &= 0b01111111;
    } else {
        reg_val |= 0b10000000;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_set_clock_invert(enum si5351_clock clk, uint8_t inv)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * inv - Set to 1 to enable, 0 to disable
 *
 * Enable to invert the clock output waveform.
 */
void si5351_set_clock_invert(enum si5351_clock clk, uint8_t inv) {
    uint8_t reg_val;
    si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val);

    if (inv == 1) {
        reg_val |= (SI5351_CLK_INVERT);
    } else {
        reg_val &= ~(SI5351_CLK_INVERT);
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_set_clock_source(enum si5351_clock clk, enum si5351_clock_source src)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * src - Which clock source to use for the multisynth
 *   (use the si5351_clock_source enum)
 *
 * Set the clock source for a multisynth (based on the options
 * presented for Registers 16-23 in the Silicon Labs AN619 document).
 * Choices are XTAL, CLKIN, MS0, or the multisynth associated with
 * the clock output.
 */
void si5351_set_clock_source(enum si5351_clock clk,
                             enum si5351_clock_source src) {
    uint8_t reg_val;
    si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk, &reg_val);

    // Clear the bits first
    reg_val &= ~(SI5351_CLK_INPUT_MASK);

    switch (src) {
        case SI5351_CLK_SRC_XTAL:
            reg_val |= (SI5351_CLK_INPUT_XTAL);
            break;
        case SI5351_CLK_SRC_CLKIN:
            reg_val |= (SI5351_CLK_INPUT_CLKIN);
            break;
        case SI5351_CLK_SRC_MS0:
            if (clk == SI5351_CLK0) {
                return;
            }

            reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_0_4);
            break;
        case SI5351_CLK_SRC_MS:
            reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_N);
            break;
        default:
            return;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
}

/*
 * si5351_set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable
 * dis_state)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * dis_state - Desired state of the output upon disable
 *   (use the si5351_clock_disable enum)
 *
 * Set the state of the clock output when it is disabled. Per page 27
 * of AN619 (Registers 24 and 25), there are four possible values: low,
 * high, high impedance, and never disabled.
 */
void si5351_set_clock_disable(enum si5351_clock clk,
                              enum si5351_clock_disable dis_state) {
    uint8_t reg_val, reg;

    if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg = SI5351_CLK3_0_DISABLE_STATE;
    } else if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg = SI5351_CLK7_4_DISABLE_STATE;
    }

    si5351_read(reg, &reg_val);

    if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg_val &= ~(0b11 << (clk * 2));
        reg_val |= dis_state << (clk * 2);
    } else if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg_val &= ~(0b11 << ((clk - 4) * 2));
        reg_val |= dis_state << ((clk - 4) * 2);
    }

    si5351_write(reg, reg_val);
}

/*
 * si5351_set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable)
 *
 * fanout - Desired clock fanout
 *   (use the si5351_clock_fanout enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Use this function to enable or disable the clock fanout options
 * for individual clock outputs. If you intend to output the XO or
 * CLKIN on the clock outputs, enable this first.
 *
 * By default, only the Multisynth fanout is enabled at startup.
 */
void si5351_set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable) {
    uint8_t reg_val;
    si5351_read(SI5351_FANOUT_ENABLE, &reg_val);

    switch (fanout) {
        case SI5351_FANOUT_CLKIN:
            if (enable) {
                reg_val |= SI5351_CLKIN_ENABLE;
            } else {
                reg_val &= ~(SI5351_CLKIN_ENABLE);
            }
            break;
        case SI5351_FANOUT_XO:
            if (enable) {
                reg_val |= SI5351_XTAL_ENABLE;
            } else {
                reg_val &= ~(SI5351_XTAL_ENABLE);
            }
            break;
        case SI5351_FANOUT_MS:
            if (enable) {
                reg_val |= SI5351_MULTISYNTH_ENABLE;
            } else {
                reg_val &= ~(SI5351_MULTISYNTH_ENABLE);
            }
            break;
    }

    si5351_write(SI5351_FANOUT_ENABLE, reg_val);
}

void si5351_update_sys_status(struct Si5351Status *status) {
    uint8_t reg_val = 0;

    if (si5351_read(SI5351_DEVICE_STATUS, &reg_val) != 0) {
        return;
    }

    /* Parse the register */
    status->SYS_INIT = (reg_val >> 7) & 0x01;
    status->LOL_B = (reg_val >> 6) & 0x01;
    status->LOL_A = (reg_val >> 5) & 0x01;
    status->LOS = (reg_val >> 4) & 0x01;
    status->REVID = reg_val & 0x03;
}

void si5351_update_int_status(struct Si5351IntStatus *int_status) {
    uint8_t reg_val = 0;

    if (si5351_read(SI5351_DEVICE_STATUS, &reg_val) != 0) {
        return;
    }

    /* Parse the register */
    int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
    int_status->LOL_B_STKY = (reg_val >> 6) & 0x01;
    int_status->LOL_A_STKY = (reg_val >> 5) & 0x01;
    int_status->LOS_STKY = (reg_val >> 4) & 0x01;
}

void si5351_set_ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4) {
    uint8_t reg_val, reg_addr = 0;

    switch (clk) {
        case SI5351_CLK0:
            reg_addr = SI5351_CLK0_PARAMETERS + 2;
            break;
        case SI5351_CLK1:
            reg_addr = SI5351_CLK1_PARAMETERS + 2;
            break;
        case SI5351_CLK2:
            reg_addr = SI5351_CLK2_PARAMETERS + 2;
            break;
        case SI5351_CLK3:
            reg_addr = SI5351_CLK3_PARAMETERS + 2;
            break;
        case SI5351_CLK4:
            reg_addr = SI5351_CLK4_PARAMETERS + 2;
            break;
        case SI5351_CLK5:
            reg_addr = SI5351_CLK5_PARAMETERS + 2;
            break;
        case SI5351_CLK6:
            return;
        case SI5351_CLK7:
            return;
    }

    si5351_read(reg_addr, &reg_val);

    // Clear the relevant bits
    reg_val &= ~(0x7c);

    if (div_by_4 == 0) {
        reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
    } else {
        reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
    }

    reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);

    si5351_write(reg_addr, reg_val);
}

uint8_t si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data) {
    if (i2c_start(SI5351_BUS_BASE_ADDR | I2C_WRITE))
        return 1;

    i2c_write(addr);

    for (int i = 0; i < bytes; i++) {
        i2c_write(data[i]);
    }

    i2c_stop();

    return 0;
}

uint8_t si5351_write(uint8_t addr, uint8_t data) {
    return si5351_write_bulk(addr, 1, &data);
}

uint8_t si5351_read(uint8_t addr, uint8_t *data) {
    if (i2c_start(SI5351_BUS_BASE_ADDR | I2C_WRITE))
        return 1;
    i2c_write(addr);

    if (i2c_start(SI5351_BUS_BASE_ADDR | I2C_READ))
        return 1;

    *data = i2c_read_nack();

    return 0;
}
