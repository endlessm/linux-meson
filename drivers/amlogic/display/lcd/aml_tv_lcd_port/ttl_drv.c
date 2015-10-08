
void set_ttl_pinmux(void)
{
	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_0, 0x3f, 0, 6);
	//aml_set_reg32_bits(P_PERIPHS_PIN_MUX_0, 0x3, 18, 2);
	//aml_set_reg32_bits(P_PERIPHS_PIN_MUX_0, 0x1, 29, 1);
	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_8, 0x7, 22, 3);
	aml_set_reg32_bits(P_PERIPHS_PIN_MUX_8, 0x1, 28, 1);

	//aml_set_reg32_bits(P_PAD_PULL_UP_EN_REG4, 0x1, 26, 1);
	//aml_set_reg32_bits(P_PAD_PULL_UP_REG4, 0x1, 26, 1);
}

static void set_tcon_ttl(Lcd_Config_t *pConf)
{
	Lcd_Timing_t *tcon_adr = &(pConf->lcd_timing);

	//set_lcd_gamma_table_ttl(pConf->lcd_effect.GammaTableR, LCD_H_SEL_R);
	//set_lcd_gamma_table_ttl(pConf->lcd_effect.GammaTableG, LCD_H_SEL_G);
	//set_lcd_gamma_table_ttl(pConf->lcd_effect.GammaTableB, LCD_H_SEL_B);

	aml_write_reg32(P_L_GAMMA_CNTL_PORT, 0);//pConf->lcd_effect.gamma_cntl_port);
	//aml_write_reg32(P_GAMMA_VCOM_HSWITCH_ADDR, pConf->lcd_effect.gamma_vcom_hswitch_addr);

	aml_write_reg32(P_L_RGB_BASE_ADDR,   0xf0);//pConf->lcd_effect.rgb_base_addr);
	aml_write_reg32(P_L_RGB_COEFF_ADDR,  0x74a);//pConf->lcd_effect.rgb_coeff_addr);
	//aml_write_reg32(P_POL_CNTL_ADDR,   pConf->lcd_timing.pol_cntl_addr);
	if (pConf->lcd_control.lvds_config->lcd_bits == 8)
		aml_write_reg32(P_L_DITH_CNTL_ADDR,  0x400);
	else
		aml_write_reg32(P_L_DITH_CNTL_ADDR,  0x600);

	aml_write_reg32(P_L_STH1_HS_ADDR,    tcon_adr->sth1_hs_addr);
	aml_write_reg32(P_L_STH1_HE_ADDR,    tcon_adr->sth1_he_addr);
	aml_write_reg32(P_L_STH1_VS_ADDR,    tcon_adr->sth1_vs_addr);
	aml_write_reg32(P_L_STH1_VE_ADDR,    tcon_adr->sth1_ve_addr);

	aml_write_reg32(P_L_OEH_HS_ADDR,     tcon_adr->oeh_hs_addr);
	aml_write_reg32(P_L_OEH_HE_ADDR,     tcon_adr->oeh_he_addr);
	aml_write_reg32(P_L_OEH_VS_ADDR,     tcon_adr->oeh_vs_addr);
	aml_write_reg32(P_L_OEH_VE_ADDR,     tcon_adr->oeh_ve_addr);

	aml_write_reg32(P_L_VCOM_HSWITCH_ADDR, tcon_adr->vcom_hswitch_addr);
	aml_write_reg32(P_L_VCOM_VS_ADDR,    tcon_adr->vcom_vs_addr);
	aml_write_reg32(P_L_VCOM_VE_ADDR,    tcon_adr->vcom_ve_addr);

	aml_write_reg32(P_L_CPV1_HS_ADDR,    tcon_adr->cpv1_hs_addr);
	aml_write_reg32(P_L_CPV1_HE_ADDR,    tcon_adr->cpv1_he_addr);
	aml_write_reg32(P_L_CPV1_VS_ADDR,    tcon_adr->cpv1_vs_addr);
	aml_write_reg32(P_L_CPV1_VE_ADDR,    tcon_adr->cpv1_ve_addr);

	aml_write_reg32(P_L_STV1_HS_ADDR,    tcon_adr->stv1_hs_addr);
	aml_write_reg32(P_L_STV1_HE_ADDR,    tcon_adr->stv1_he_addr);
	aml_write_reg32(P_L_STV1_VS_ADDR,    tcon_adr->stv1_vs_addr);
	aml_write_reg32(P_L_STV1_VE_ADDR,    tcon_adr->stv1_ve_addr);

	aml_write_reg32(P_L_OEV1_HS_ADDR,    tcon_adr->oev1_hs_addr);
	aml_write_reg32(P_L_OEV1_HE_ADDR,    tcon_adr->oev1_he_addr);
	aml_write_reg32(P_L_OEV1_VS_ADDR,    tcon_adr->oev1_vs_addr);
	aml_write_reg32(P_L_OEV1_VE_ADDR,    tcon_adr->oev1_ve_addr);

	aml_write_reg32(P_L_INV_CNT_ADDR,    tcon_adr->inv_cnt_addr);
	aml_write_reg32(P_L_TCON_MISC_SEL_ADDR,     tcon_adr->tcon_misc_sel_addr);
	aml_write_reg32(P_L_DUAL_PORT_CNTL_ADDR, tcon_adr->dual_port_cntl_addr);

	aml_write_reg32(P_VPP_MISC, aml_read_reg32(P_VPP_MISC) & ~(VPP_OUT_SATURATE));
}

static void venc_set_ttl(Lcd_Config_t *pConf)
{
	printf("%s\n", __FUNCTION__);
	aml_write_reg32(P_ENCL_VIDEO_EN,           0);
	aml_write_reg32(P_VPU_VIU_VENC_MUX_CTRL,
	   (0<<0) |    // viu1 select ENCL
	   (0<<2)      // viu2 select ENCL
	);
	aml_write_reg32(P_ENCL_VIDEO_MODE,        0);
	aml_write_reg32(P_ENCL_VIDEO_MODE_ADV,    0x0418);

	// bypass filter
	aml_write_reg32(P_ENCL_VIDEO_FILT_CTRL,    0x1000);
	aml_write_reg32(P_VENC_DVI_SETTING,        0x11);
	aml_write_reg32(P_VENC_VIDEO_PROG_MODE,    0x100);

	aml_write_reg32(P_ENCL_VIDEO_MAX_PXCNT,    pConf->lcd_basic.h_period - 1);
	aml_write_reg32(P_ENCL_VIDEO_MAX_LNCNT,    pConf->lcd_basic.v_period - 1);

	aml_write_reg32(P_ENCL_VIDEO_HAVON_BEGIN,  pConf->lcd_basic.video_on_pixel);
	aml_write_reg32(P_ENCL_VIDEO_HAVON_END,    pConf->lcd_basic.h_active - 1 + pConf->lcd_basic.video_on_pixel);
	aml_write_reg32(P_ENCL_VIDEO_VAVON_BLINE,  pConf->lcd_basic.video_on_line);
	aml_write_reg32(P_ENCL_VIDEO_VAVON_ELINE,  pConf->lcd_basic.v_active + 3  + pConf->lcd_basic.video_on_line);

	aml_write_reg32(P_ENCL_VIDEO_HSO_BEGIN,    15);
	aml_write_reg32(P_ENCL_VIDEO_HSO_END,      31);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BEGIN,    15);
	aml_write_reg32(P_ENCL_VIDEO_VSO_END,      31);
	aml_write_reg32(P_ENCL_VIDEO_VSO_BLINE,    0);
	aml_write_reg32(P_ENCL_VIDEO_VSO_ELINE,    2);

	// enable ENCL
	aml_write_reg32(P_ENCL_VIDEO_EN,           1);
}

 void vclk_set_encl_ttl(vmode_t vmode)
{
	int hdmi_clk_out;
	//int hdmi_vx1_clk_od1;
	int vx1_phy_div;
	int encl_div;
	unsigned int xd;
	//no used, od2 must >= od3.
	//hdmi_vx1_clk_od1 = 1; //OD1 always 1

	//pll_video.pl 3500 pll_out
	switch (vmode) {
		case VMODE_720P: //total: 1650*750 pixel clk = 74.25MHz,phy_clk(s)=(pclk*7)= 519.75 = 1039.5/2
			hdmi_clk_out = 1039.5;
			vx1_phy_div  = 2/2;
			encl_div     = vx1_phy_div*7;
			break;

		default:
			printf("Error video format!\n");
			return;
	}
	//if(set_hdmi_dpll(hdmi_clk_out,hdmi_vx1_clk_od1)) {
	if (set_hdmi_dpll(hdmi_clk_out,0)) {
		printf("Unsupported HDMI_DPLL out frequency!\n");
		return;
	}

	//configure vid_clk_div_top
	if ((encl_div%14) == 0) {//7*even
		clocks_set_vid_clk_div(CLK_UTIL_VID_PLL_DIV_14);
		xd = encl_div/14;
	}else if ((encl_div%7) == 0) { //7*odd
		clocks_set_vid_clk_div(CLK_UTIL_VID_PLL_DIV_7);
		xd = encl_div/7;
	}else{ //3.5*odd
		clocks_set_vid_clk_div(CLK_UTIL_VID_PLL_DIV_3p5);
		xd = encl_div/3.5;
	}

	//configure crt_video
	set_crt_video_enc(0,0,xd);  //configure crt_video V1: inSel=vid_pll_clk(0),DivN=xd)
	enable_crt_video_encl(1,0); //select and enable the output
}

static void set_pll_ttl(Lcd_Config_t *pConf)
{
	vclk_set_encl_ttl(pDev->lcd_info.mode);
//	set_video_spread_spectrum(pll_sel, ss_level);
}



