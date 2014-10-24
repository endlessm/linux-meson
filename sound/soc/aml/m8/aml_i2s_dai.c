#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/soundcard.h>
#include <linux/timer.h>
#include <linux/debugfs.h>
#include <linux/major.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include "aml_i2s_dai.h"
#include "aml_pcm.h"
#include "aml_i2s.h"
#include "aml_audio_hw.h"
#include <linux/of.h>

static aml_dai_info_t dai_info[3] = {{0}};
static int i2s_pos_sync = 0;
//#define AML_DAI_DEBUG
//#define AML_DAI_PCM_SUPPORT 


#define ALSA_PRINT(fmt,args...)	printk(KERN_INFO "[aml-i2s-dai]" fmt,##args)
#ifdef DEBUG_ALSA_SOC_DAI_SPDIF
#define ALSA_DEBUG(fmt,args...) 	printk(KERN_INFO "[aml-i2s-dai]" fmt,##args)
#define ALSA_TRACE()     			printk("[aml-i2s-dai] enter func %s,line %d\n",__FUNCTION__,__LINE__)
#else
#define ALSA_DEBUG(fmt,args...) 
#define ALSA_TRACE()   
#endif

/*
the I2S hw  and IEC958 PCM output initation,958 initation here,
for the case that only use our ALSA driver for PCM s/pdif output.
*/
static void  aml_hw_i2s_init(struct snd_pcm_runtime *runtime)
{

		unsigned i2s_mode = AIU_I2S_MODE_PCM16;
		switch(runtime->format){
		case SNDRV_PCM_FORMAT_S32_LE:
			i2s_mode = AIU_I2S_MODE_PCM32;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			i2s_mode = AIU_I2S_MODE_PCM24;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			i2s_mode = AIU_I2S_MODE_PCM16;
			break;
		}
		audio_set_i2s_mode(i2s_mode);
		audio_set_aiubuf(runtime->dma_addr, runtime->dma_bytes,runtime->channels);
		ALSA_PRINT("i2s dma %x,phy addr %x,mode %d,ch %d \n",(unsigned)runtime->dma_area,(unsigned)runtime->dma_addr,i2s_mode,runtime->channels);

}
static int aml_dai_i2s_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{	  	
	int ret = 0;
    	struct snd_pcm_runtime *runtime = substream->runtime;
    	struct aml_runtime_data *prtd = (struct aml_runtime_data *)runtime->private_data;
	audio_stream_t *s;	
    ALSA_TRACE();
	if(prtd == NULL){
		prtd = (struct aml_runtime_data *)kzalloc(sizeof(struct aml_runtime_data), GFP_KERNEL);
		if (prtd == NULL) {
			printk("alloc aml_runtime_data error\n");
			ret = -ENOMEM;
			goto out;
		}
		prtd->substream = substream;
		runtime->private_data = prtd;		
	}
	s = &prtd->s; 
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		s->device_type = AML_AUDIO_I2SOUT;
	}	
	else{
		s->device_type = AML_AUDIO_I2SIN;	
	}	
	return 0;
out:
	return ret;
}
extern void  aml_spdif_play();
static void aml_dai_i2s_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	ALSA_TRACE();
    struct snd_pcm_runtime *runtime = substream->runtime;
    if(runtime->channels == 8){
        aml_spdif_play();
    }
}
static int  set_clock = -1;
#define AOUT_EVENT_IEC_60958_PCM 0x1
extern int aout_notifier_call_chain(unsigned long val,void * v);

static int aml_dai_i2s_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{	
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_runtime_data *prtd = runtime->private_data;
	int  sample_rate = AUDIO_CLK_FREQ_48;
	audio_stream_t *s = &prtd->s;	
    ALSA_TRACE();
	switch(runtime->rate){
		case 192000:
			sample_rate	=	AUDIO_CLK_FREQ_192;
			break;
		case 176400:
			sample_rate	=	AUDIO_CLK_FREQ_1764;
			break;
		case 96000:
			sample_rate	=	AUDIO_CLK_FREQ_96;
			break;
		case 88200:
			sample_rate	=	AUDIO_CLK_FREQ_882;
			break;
		case 48000:
			sample_rate	=	AUDIO_CLK_FREQ_48;
			break;
		case 44100:
			sample_rate	=	AUDIO_CLK_FREQ_441;
			break;
		case 32000:
			sample_rate	=	AUDIO_CLK_FREQ_32;
			break;
		case 8000:
			sample_rate	=	AUDIO_CLK_FREQ_8;
			break;
		case 11025:
			sample_rate	=	AUDIO_CLK_FREQ_11;
			break;
		case 16000:
			sample_rate	=	AUDIO_CLK_FREQ_16;
			break;
		case 22050:
			sample_rate	=	AUDIO_CLK_FREQ_22;
			break;
		case 12000:
			sample_rate	=	AUDIO_CLK_FREQ_12;
			break;
		case 24000:
			sample_rate	=	AUDIO_CLK_FREQ_22;
			break;
		default:
			sample_rate	=	AUDIO_CLK_FREQ_441;
			break;
	};

    if(set_clock != sample_rate ){
		ALSA_PRINT("enterd %s,set_clock:%d,sample_rate=%d\n",__func__,set_clock,sample_rate);
        set_clock = sample_rate;
        audio_set_i2s_clk(sample_rate, AUDIO_CLK_256FS);
    }
    audio_util_set_dac_i2s_format(AUDIO_ALGOUT_DAC_FORMAT_DSP); 
    
    if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        s->i2s_mode = dai_info[dai->id].i2s_mode;
        audio_in_i2s_set_buf(runtime->dma_addr, runtime->dma_bytes*2,0,i2s_pos_sync);
        memset((void*)runtime->dma_area,0,runtime->dma_bytes*2);
        {
            int * ppp = (int*)(runtime->dma_area+runtime->dma_bytes*2-8);
            ppp[0] = 0x78787878;
            ppp[1] = 0x78787878;
        }
        s->device_type = AML_AUDIO_I2SIN;       
    }
    else{       
        s->device_type = AML_AUDIO_I2SOUT;
        aml_hw_i2s_init(runtime);
    }
    if(runtime->channels == 8){
        printk("[%s,%d]8ch PCM output->notify HDMI\n",__FUNCTION__,__LINE__);
        aout_notifier_call_chain(AOUT_EVENT_IEC_60958_PCM,substream);
    }
    return 0;
}
static int aml_dai_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
    int * ppp = NULL;
    ALSA_TRACE();
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			// TODO
			if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				printk("aiu i2s playback enable\n\n");
				audio_out_i2s_enable(1);
			}else{
				audio_in_i2s_enable(1);
				ppp = (int*)(rtd->dma_area+rtd->dma_bytes*2-8);
				ppp[0] = 0x78787878;
				ppp[1] = 0x78787878;
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				printk("aiu i2s playback disable\n\n");
				audio_out_i2s_enable(0);
			}else{
				audio_in_i2s_enable(0);
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}	

static int aml_dai_i2s_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
	ALSA_TRACE();
	return 0;
}

static int aml_dai_set_i2s_fmt(struct snd_soc_dai *dai,
					unsigned int fmt)
{
	ALSA_TRACE();
	if(fmt&SND_SOC_DAIFMT_CBS_CFS)//slave mode 
		dai_info[dai->id].i2s_mode = I2S_SLAVE_MODE;
    
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:
         i2s_pos_sync = 0;
        break;
    case SND_SOC_DAIFMT_IB_NF:
         i2s_pos_sync = 1;
        break;
    default:
        return -EINVAL;
    }
	return 0;
}

static int aml_dai_set_i2s_sysclk(struct snd_soc_dai *dai,
					int clk_id, unsigned int freq, int dir)
{
	ALSA_TRACE();
	return 0;
}

#ifdef CONFIG_PM
static int aml_dai_i2s_suspend(struct snd_soc_dai *dai)
{
	ALSA_TRACE();
	return 0;
}

static int aml_dai_i2s_resume(struct snd_soc_dai *dai)
{
	ALSA_TRACE();
	return 0;
}

#else /* CONFIG_PM */
#define aml_dai_i2s_suspend	NULL
#define aml_dai_i2s_resume	NULL
#endif /* CONFIG_PM */



#define AML_DAI_I2S_RATES		(SNDRV_PCM_RATE_8000_96000)
#define AML_DAI_I2S_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


static struct snd_soc_dai_ops aml_dai_i2s_ops = {
	.startup	= aml_dai_i2s_startup,
	.shutdown	= aml_dai_i2s_shutdown,
	.prepare	= aml_dai_i2s_prepare,
	.trigger = aml_dai_i2s_trigger,
	.hw_params	= aml_dai_i2s_hw_params,
	.set_fmt	= aml_dai_set_i2s_fmt,
	.set_sysclk	= aml_dai_set_i2s_sysclk,
};



struct snd_soc_dai_driver aml_i2s_dai[] = {
	{	.name = "aml-i2s-dai",
		.id = 0,
		.suspend = aml_dai_i2s_suspend,
		.resume = aml_dai_i2s_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = AML_DAI_I2S_RATES,
			.formats = AML_DAI_I2S_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = AML_DAI_I2S_RATES,
			.formats = AML_DAI_I2S_FORMATS,},
		.ops = &aml_dai_i2s_ops,
	},

};

EXPORT_SYMBOL_GPL(aml_i2s_dai);

static const struct snd_soc_component_driver aml_component= {
	.name		= "aml-i2s-dai",
};
static int aml_i2s_dai_probe(struct platform_device *pdev)
{
	printk(KERN_DEBUG "enter %s\n", __func__);
#if 0
	BUG_ON(pdev->id < 0);
	BUG_ON(pdev->id >= ARRAY_SIZE(aml_dai));
	return snd_soc_register_dai(&pdev->dev, &aml_dai[pdev->id]);
#else
	return snd_soc_register_component(&pdev->dev, &aml_component,
					 aml_i2s_dai, ARRAY_SIZE(aml_i2s_dai));

#endif
}

static int aml_i2s_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id amlogic_dai_dt_match[]={
	{	.compatible = "amlogic,aml-i2s-dai",
	},
	{},
};
#else
#define amlogic_dai_dt_match NULL
#endif

static struct platform_driver aml_i2s_dai_driver = {
	.driver = {
		.name = "aml-i2s-dai",
		.owner = THIS_MODULE,
		.of_match_table = amlogic_dai_dt_match,
	},

	.probe = aml_i2s_dai_probe,
	.remove = aml_i2s_dai_remove,
};

static int __init aml_i2s_dai_modinit(void)
{
	return platform_driver_register(&aml_i2s_dai_driver);
}
module_init(aml_i2s_dai_modinit);

static void __exit aml_i2s_dai_modexit(void)
{
	platform_driver_unregister(&aml_i2s_dai_driver);
}
module_exit(aml_i2s_dai_modexit);

/* Module information */
MODULE_AUTHOR("AMLogic, Inc.");
MODULE_DESCRIPTION("AML DAI driver for ALSA");
MODULE_LICENSE("GPL");
