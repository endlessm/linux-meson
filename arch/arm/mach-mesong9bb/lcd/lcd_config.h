
#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H
#include <linux/types.h>

/* **********************************
//lcd driver version
// ********************************** */
#define LCD_DRV_TYPE      "c9b"
#define LCD_DRV_DATE      "20150915"
/* ********************************** */

/* for GAMMA_CNTL_PORT */
	#define LCD_GAMMA_VCOM_POL       7
	#define LCD_GAMMA_RVS_OUT        6
	#define LCD_ADR_RDY              5
	#define LCD_WR_RDY               4
	#define LCD_RD_RDY               3
	#define LCD_GAMMA_TR             2
	#define LCD_GAMMA_SET            1
	#define LCD_GAMMA_EN             0

/* for GAMMA_ADDR_PORT */
	#define LCD_H_RD                 12
	#define LCD_H_AUTO_INC           11
	#define LCD_H_SEL_R              10
	#define LCD_H_SEL_G              9
	#define LCD_H_SEL_B              8
	#define LCD_HADR_MSB             7
	#define LCD_HADR                 0

/* for POL_CNTL_ADDR */
	#define LCD_DCLK_SEL             14
	#define LCD_TCON_VSYNC_SEL_DVI   11
	#define LCD_TCON_HSYNC_SEL_DVI   10
	#define LCD_TCON_DE_SEL_DVI      9
	#define LCD_CPH3_POL             8
	#define LCD_CPH2_POL             7
	#define LCD_CPH1_POL             6
	#define LCD_TCON_DE_SEL          5
	#define LCD_TCON_VS_SEL          4
	#define LCD_TCON_HS_SEL          3
	#define LCD_DE_POL               2
	#define LCD_VS_POL               1
	#define LCD_HS_POL               0

/* for DITH_CNTL_ADDR */
	#define LCD_DITH10_EN            10
	#define LCD_DITH8_EN             9
	#define LCD_DITH_MD              8
	#define LCD_DITH10_CNTL_MSB      7
	#define LCD_DITH10_CNTL          4
	#define LCD_DITH8_CNTL_MSB       3
	#define LCD_DITH8_CNTL           0

/* for INV_CNT_ADDR */
	#define LCD_INV_EN               4
	#define LCD_INV_CNT_MSB          3
	#define LCD_INV_CNT              0

/* for TCON_MISC_SEL_ADDR */
	#define LCD_STH2_SEL             12
	#define LCD_STH1_SEL             11
	#define LCD_OEH_SEL              10
	#define LCD_VCOM_SEL             9
	#define LCD_DB_LINE_SW           8
	#define LCD_CPV2_SEL             7
	#define LCD_CPV1_SEL             6
	#define LCD_STV2_SEL             5
	#define LCD_STV1_SEL             4
	#define LCD_OEV_UNITE            3
	#define LCD_OEV3_SEL             2
	#define LCD_OEV2_SEL             1
	#define LCD_OEV1_SEL             0

/* for DUAL_PORT_CNTL_ADDR */
	#define LCD_ANALOG_SEL_CPH3      8
	#define LCD_ANALOG_3PHI_CLK_SEL  7
	#define LCD_LVDS_SEL54           6
	#define LCD_LVDS_SEL27           5
	#define LCD_TTL_SEL              4
	#define LCD_DUAL_PIXEL           3
	#define LCD_PORT_SWP             2
	#define LCD_RGB_SWP              1
	#define LCD_BIT_SWP              0

/* for LVDS_PACK_CNTL_ADDR */
	#define LCD_LD_CNT_MSB           7
	#define LCD_LD_CNT               5
	#define LCD_PN_SWP               4
	#define LCD_RES                  3
	#define LCD_LVDS_PORT_SWP        2
	#define LCD_PACK_RVS             1
	#define LCD_PACK_LITTLE          0

/* for LVDS_BLANK_DATA */
	#define LVDS_BLANK_DATA_RESERVED    30
	#define LVDS_BLANK_DATA_R           20
	#define LVDS_BLANK_DATA_G           10
	#define LVDS_BLANK_DATA_B           0

/* for LVDS_PACK_CNTL_ADDR */
	#define LVDS_USE_TCON               7
	#define LVDS_DUAL                   6
	#define PN_SWP                      5
	#define LSB_FIRST                   4
	#define LVDS_RESV                   3
	#define ODD_EVEN_SWP                2
	#define LVDS_REPACK                 0

static const unsigned gamma_sel_table[3] = {
	LCD_H_SEL_R,
	LCD_H_SEL_G,
	LCD_H_SEL_B,
};

/* ******************************************** */
/* for video encoder */
	#define LVDS_DELAY			8
	#define TTL_DELAY			19

/* ********************************************
// for clk parameter auto generation
// ********************************************* */
/**** clk parameters bit ***/
	/* pll_ctrl */
	#define PLL_CTRL_LOCK			31
	#define PLL_CTRL_EN			30
	#define PLL_CTRL_RST			28
	#define PLL_CTRL_OD2			22 /* [23:22] */
	#define PLL_CTRL_OD3			18 /* [19:18] */
	#define PLL_CTRL_OD1			16 /* [17:16] */
	#define PLL_CTRL_N			9 /* [13:9] */
	#define PLL_CTRL_M			0 /* [8:0] */

	/* div_ctrl */
	#define DIV_CTRL_CLK_DIV		0 /* [7:0] */

	/* clk_ctrl */
	#define CLK_CTRL_FRAC			16 /* [27:16] */

	#define PLL_WAIT_LOCK_CNT		200

/**** clk frequency limit ***/
	/* PLL */
	#define PLL_M_MIN			2
	#define PLL_M_MAX			511
	#define PLL_N_MIN			1
	#define PLL_N_MAX			1
	#define PLL_FREF_MIN			(5 * 1000)
	#define PLL_FREF_MAX			(25 * 1000)
	#define PLL_VCO_MIN			(3000 * 1000)
	#define PLL_VCO_MAX			(6000 * 1000)
	/* VIDEO */
	#define CLK_DIV_MAX_CLK_IN		(3000 * 1000)
	#define CRT_VID_MAX_CLK_IN		(3000 * 1000)
	#define ENCL_MAX_CLK_IN			(333 * 1000)

	/* lcd interface video clk */
	#define LVDS_MAX_VID_CLK_IN		ENCL_MAX_CLK_IN
	#define TTL_MAX_VID_CLK_IN		ENCL_MAX_CLK_IN

	/* clk max error */
	#define MAX_ERROR			(2 * 1000)

#define CRT_VID_DIV_MAX				255

#define OD_SEL_MAX				3

static unsigned od_table[4] = {1, 2, 4, 8};

/* g9tv, g9bb divider */
#define CLK_DIV_I2O     0
#define CLK_DIV_O2I     1
enum div_sel_e {
	CLK_DIV_SEL_1 = 0,
	CLK_DIV_SEL_2,    /* 1 */
	CLK_DIV_SEL_3,    /* 2 */
	CLK_DIV_SEL_3p5,  /* 3 */
	CLK_DIV_SEL_3p75, /* 4 */
	CLK_DIV_SEL_4,    /* 5 */
	CLK_DIV_SEL_5,    /* 6 */
	CLK_DIV_SEL_6,    /* 7 */
	CLK_DIV_SEL_6p25, /* 8 */
	CLK_DIV_SEL_7,    /* 9 */
	CLK_DIV_SEL_7p5,  /* 10 */
	CLK_DIV_SEL_12,   /* 11 */
	CLK_DIV_SEL_14,   /* 12 */
	CLK_DIV_SEL_15,   /* 13 */
	CLK_DIV_SEL_2p5,  /* 14 */
	CLK_DIV_SEL_MAX,
};
/* ******************************************** */

/* ********************************************
//DPHY Config
// ******************************************** */
/* ******** G9TV,G9BB ******** */
/* bit[15:11] */
#define BIT_PHY_LANE         16
#define WIDTH_PHY_LANE       12

/* LVDS */
#define LVDS_LANE_A0          (1 << 0)
#define LVDS_LANE_A1          (1 << 1)
#define LVDS_LANE_A2          (1 << 2)
#define LVDS_LANE_ACLK        (1 << 3)
#define LVDS_LANE_A3          (1 << 4)
#define LVDS_LANE_A4          (1 << 5)
#define LVDS_LANE_B0          (1 << 6)
#define LVDS_LANE_B1          (1 << 7)
#define LVDS_LANE_B2          (1 << 8)
#define LVDS_LANE_BCLK        (1 << 9)
#define LVDS_LANE_B3          (1 << 10)
#define LVDS_LANE_B4          (1 << 11)
#define LVDS_PORT_A           (LVDS_LANE_ACLK | LVDS_LANE_A0 |\
				LVDS_LANE_A1 | LVDS_LANE_A2 |\
				LVDS_LANE_A3 | LVDS_LANE_A4)
#define LVDS_PORT_B           (LVDS_LANE_BCLK | LVDS_LANE_B0 |\
				LVDS_LANE_B1 | LVDS_LANE_B2 |\
				LVDS_LANE_B3 | LVDS_LANE_B4)
#define LVDS_PORT_AB          (LVDS_PORT_A | LVDS_PORT_B)

/* VBYONE */
#define VBYONE_LANE_0         (1 << 4)
#define VBYONE_LANE_1         (1 << 3)
#define VBYONE_LANE_2         (1 << 1)
#define VBYONE_LANE_3         (1 << 0)
#define VBYONE_LANE_4         (1 << 2)
#define VBYONE_LANE_5         (1 << 3)
#define VBYONE_LANE_6         (1 << 1)
#define VBYONE_LANE_7         (1 << 0)
#define VBYONE_PORT           (VBYONE_LANE_0 | VBYONE_LANE_1 |\
				VBYONE_LANE_2 | VBYONE_LANE_3 |\
				VBYONE_LANE_4 | VBYONE_LANE_5 |\
				VBYONE_LANE_6 | VBYONE_LANE_7)

/* ******************************************** */

#endif
