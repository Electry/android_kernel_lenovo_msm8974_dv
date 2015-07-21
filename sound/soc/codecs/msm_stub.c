/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static int msm_stub_dai_startup(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    return 0;
}

static int msm_stub_dai_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
    return 0;
}

static void msm_stub_dai_shutdown(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    return;
}

static int msm_stub_dai_sysclk(struct snd_soc_dai *dai,
        int clk_id, unsigned int freq, int dir)
{
    return 0;
}

static int msm_stub_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    return 0;
}

static struct snd_soc_dai_ops msm_stub_dai_ops = {
    .startup = msm_stub_dai_startup,
    .hw_params = msm_stub_dai_hw_params,
    .shutdown = msm_stub_dai_shutdown,
    .set_sysclk = msm_stub_dai_sysclk,
    .set_fmt = msm_stub_dai_fmt,
};

/* A dummy driver useful only to advertise hardware parameters */
static struct snd_soc_dai_driver msm_stub_dais[] = {
	{
		.name = "msm-stub-rx",
		.playback = { /* Support maximum range */
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
        .ops = &msm_stub_dai_ops,
	},
	{
		.name = "msm-stub-tx",
		.capture = { /* Support maximum range */
			.stream_name = "Record",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
		},
        .ops = &msm_stub_dai_ops,
	},
};

static struct snd_soc_codec_driver soc_msm_stub = {};

static int __devinit msm_stub_dev_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s.%d", "msm-stub-codec", 1);

	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));

	return snd_soc_register_codec(&pdev->dev,
	&soc_msm_stub, msm_stub_dais, ARRAY_SIZE(msm_stub_dais));
}

static int __devexit msm_stub_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}
static const struct of_device_id msm_stub_codec_dt_match[] = {
	{ .compatible = "qcom,msm-stub-codec", },
	{}
};

static struct platform_driver msm_stub_driver = {
	.driver = {
		.name = "msm-stub-codec",
		.owner = THIS_MODULE,
		.of_match_table = msm_stub_codec_dt_match,
	},
	.probe = msm_stub_dev_probe,
	.remove = __devexit_p(msm_stub_dev_remove),
};

static int __init msm_stub_init(void)
{
	return platform_driver_register(&msm_stub_driver);
}
module_init(msm_stub_init);

static void __exit msm_stub_exit(void)
{
	platform_driver_unregister(&msm_stub_driver);
}
module_exit(msm_stub_exit);

MODULE_DESCRIPTION("Generic MSM CODEC driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
