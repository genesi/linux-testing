/*
 * Freescale iMX PATA driver
 *
 * Copyright (C) 2011 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 * Based on pata_platform - Copyright (C) 2006 - 2007  Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * TODO:
 * - dmaengine support
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <scsi/scsi_host.h>
#include <linux/ata.h>
#include <linux/libata.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>

#define DRV_NAME "pata_imx"

#define PATA_IMX_FIFO_DATA		0x1C
#define PATA_IMX_ATA_CONTROL		0x24
#define PATA_IMX_ATA_CTRL_FIFO_RST_B	(1<<7)
#define PATA_IMX_ATA_CTRL_ATA_RST_B	(1<<6)
#define PATA_IMX_ATA_CTRL_IORDY_EN	(1<<0)
#define PATA_IMX_ATA_INT_EN		0x2C
#define PATA_IMX_ATA_INTR_ATA_INTRQ2	(1<<3)
#define PATA_IMX_DRIVE_DATA		0xA0
#define PATA_IMX_DRIVE_CONTROL		0xD8

struct pata_imx_dma {
	int dma;
	unsigned long dma_addr;
	int burstsize;
};

struct pata_imx_priv {
	struct clk *clk;
	/* timings/interrupt/control regs */
	u8 *host_regs;
	u32 ata_ctl;
	struct pata_imx_dma dma_params_rx;
	struct pata_imx_dma dma_params_tx;
};

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 5 PIO modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short 	t0,  t1, t2_8, t2_16, t2i, t4, t9, tA;
} pio_specs[] = {
	[0] = { 600, 70, 290,  165,   0,   30, 20, 50,},
	[1] = { 383, 50, 290,  125,   0,   20, 15, 50,},
	[2] = {	240, 30, 290,  100,   0,   15, 10, 50,},
	[3] = { 180, 30, 80,    80,   0,   10, 10, 50,},
	[4] = { 120, 25, 70,    70,   0,   10, 10, 50,},
};
#define NR_PIO_SPECS (sizeof pio_specs / sizeof pio_specs[0])

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 3 MDMA modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short 	t0M, tD, tH, tJ, tKW, tM, tN, tJNH;
} mdma_specs[] = {
	[0] = {	480, 215, 20, 20, 215, 50, 15, 20,},
	[1] = { 150, 80,  15, 5,  50, 30, 10, 15,},
	[2] = {	120, 70, 10, 5, 25, 25, 10, 10,},
};
#define NR_MDMA_SPECS (sizeof mdma_specs / sizeof mdma_specs[0])

/*
 * This structure contains the timing parameters for
 * ATA bus timing in the 6 UDMA modes.  The timings
 * are in nanoseconds, and are converted to clock
 * cycles before being stored in the ATA controller
 * timing registers.
 */
static struct {
	short t2CYC, tCYC, tDS, tDH, tDVS, tDVH, tCVS, tCVH, tFS_min, tLI_max,
	    tMLI, tAZ, tZAH, tENV_min, tSR, tRFS, tRP, tACK, tSS, tDZFS;
} udma_specs[] = {
	[0] = { 235, 114,  15,   5,   70,   6,   70,    6,    0,     100,
		20,   10,   20,   20,       50,  75,   160, 20,  50,  80,},
	[1] = {	156, 75,   10,   5,   48,   6,   48,    6,    0,     100,
		20,   10,   20,   20,       30,  70,   125, 20,  50,  63,},
	[2] = {	117, 55,    7,   5,   34,   6,   34,    6,    0,     100,
		20,   10,   20,   20,       20,  60,   100, 20,  50,  47,},
	[3] = {	86, 39,     7,   5,   20,   6,   20,    6,    0,     100,
		20,   10,   20,   20,       20,  60,   100, 20,  50,  35,},
	[4] = {	57, 25,     5,   5,    7,   6,    7,    6,    0,     100,
		20,   10,   20,   20,       50,  60,   100, 20,  50,  25,},
	[5] = { 38, 17,     4,   5,    5,   6,   10,   10,    0,     75,
		20,   10,   20,   20,       20,  50,    85, 20,  50,  40,},
};
#define NR_UDMA_SPECS (sizeof udma_specs / sizeof udma_specs[0])

struct pata_imx_time_regs {
	u8 time_off, time_on, time_1, time_2w;
	u8 time_2r, time_ax, time_pio_rdx, time_4;
	u8 time_9, time_m, time_jn, time_d;
	u8 time_k, time_ack, time_env, time_rpx;
	u8 time_zah, time_mlix, time_dvh, time_dzfs;
	u8 time_dvs, time_cvh, time_ss, time_cyc;
};

static void commit_timing_config(struct pata_imx_time_regs *tp, struct ata_host *host)
{
	u32 *lp = (u32 *) tp;
	struct pata_imx_priv *priv = host->private_data;
	u32 *ctlp = (u32 *) priv->host_regs;
	int i;

	for (i = 0; i < (sizeof(struct pata_imx_time_regs)/sizeof(int)); i++) {
		__raw_writel(*lp, ctlp);
		lp++;
		ctlp++;
	}
}

/*!
 * Calculate values for the ATA bus timing registers and store
 * them into the hardware.
 *
 * @param       xfer_mode   specifies XFER xfer_mode
 * @param       pdev        specifies platform_device
 *
 * @return      EINVAL      speed out of range, or illegal mode
 */
static int set_ata_bus_timing(u8 xfer_mode, struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct pata_imx_priv *priv = host->private_data;
	int speed;

	/* get the bus clock cycle time, in ns */
	int T = 1 * 1000 * 1000 * 1000 / clk_get_rate(priv->clk);
	struct pata_imx_time_regs tr;

	/* set register values to 0x1 (power on default) */
	memset(&tr, 0x1, sizeof(struct pata_imx_time_regs));

	/* every mode gets the same t_off and t_on */
	tr.time_off = 3;
	tr.time_on = 3;

	if (xfer_mode < XFER_MW_DMA_0) {
		speed = xfer_mode - XFER_PIO_0;
		if (speed >= NR_PIO_SPECS)
			return -EINVAL;

		tr.time_1 = (pio_specs[speed].t1 + T) / T;
		tr.time_2w = (pio_specs[speed].t2_8 + T) / T;
		tr.time_2r = (pio_specs[speed].t2_8 + T) / T;
		tr.time_ax = (pio_specs[speed].tA + T) / T + 2;
		tr.time_pio_rdx = 1;
		tr.time_4 = (pio_specs[speed].t4 + T) / T;
		tr.time_9 = (pio_specs[speed].t9 + T) / T;
	} else return -EINVAL;

	commit_timing_config(&tr, host);

	return 0;
}

static void pata_imx_set_piomode(struct ata_port *ap, struct ata_device *adev)
{
	struct pata_imx_priv *priv = ap->host->private_data;
	u32 val;

	val = __raw_readl(priv->host_regs + PATA_IMX_ATA_CONTROL);
	if (ata_pio_need_iordy(adev))
		val |= PATA_IMX_ATA_CTRL_IORDY_EN;
	else
		val &= ~PATA_IMX_ATA_CTRL_IORDY_EN;
	__raw_writel(val, priv->host_regs + PATA_IMX_ATA_CONTROL);

	set_ata_bus_timing(adev->pio_mode, to_platform_device(ap->dev));
}

static struct scsi_host_template pata_imx_sht = {
	ATA_PIO_SHT(DRV_NAME),
};

static struct ata_port_operations pata_imx_port_ops = {
	.inherits			= &ata_sff_port_ops,
#ifdef CONFIG_LEDS_TRIGGER_IDE_DISK
	.qc_issue			= ata_sff_qc_issue_ledtrigger,
#endif
	.sff_data_xfer  		= ata_sff_data_xfer_noirq,
	.cable_detect			= ata_cable_unknown,
	.set_piomode			= pata_imx_set_piomode,
};

static void pata_imx_setup_port(struct ata_ioports *ioaddr)
{
	/* Fixup the port shift for platforms that need it */
	ioaddr->data_addr	= ioaddr->cmd_addr + (ATA_REG_DATA    << 2);
	ioaddr->error_addr	= ioaddr->cmd_addr + (ATA_REG_ERR     << 2);
	ioaddr->feature_addr	= ioaddr->cmd_addr + (ATA_REG_FEATURE << 2);
	ioaddr->nsect_addr	= ioaddr->cmd_addr + (ATA_REG_NSECT   << 2);
	ioaddr->lbal_addr	= ioaddr->cmd_addr + (ATA_REG_LBAL    << 2);
	ioaddr->lbam_addr	= ioaddr->cmd_addr + (ATA_REG_LBAM    << 2);
	ioaddr->lbah_addr	= ioaddr->cmd_addr + (ATA_REG_LBAH    << 2);
	ioaddr->device_addr	= ioaddr->cmd_addr + (ATA_REG_DEVICE  << 2);
	ioaddr->status_addr	= ioaddr->cmd_addr + (ATA_REG_STATUS  << 2);
	ioaddr->command_addr	= ioaddr->cmd_addr + (ATA_REG_CMD     << 2);
}

static int __devinit pata_imx_probe(struct platform_device *pdev)
{
	struct ata_host *host;
	struct ata_port *ap;
	struct pata_imx_priv *priv;
	int irq = 0;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev,
				sizeof(struct pata_imx_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	clk_enable(priv->clk);

	host = ata_host_alloc(&pdev->dev, 1);
	if (!host)
		goto free_priv;

	host->private_data = priv;
	ap = host->ports[0];

	ap->ops = &pata_imx_port_ops;
	ap->pio_mask = ATA_PIO4;
	ap->flags |= ATA_FLAG_SLAVE_POSS;

	priv->host_regs = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!priv->host_regs) {
		dev_err(&pdev->dev, "failed to map IO/CTL base\n");
		goto free_priv;
	}

	ap->ioaddr.cmd_addr = priv->host_regs + PATA_IMX_DRIVE_DATA;
	ap->ioaddr.ctl_addr = priv->host_regs + PATA_IMX_DRIVE_CONTROL;
	ap->ioaddr.altstatus_addr = ap->ioaddr.ctl_addr;

	pata_imx_setup_port(&ap->ioaddr);

	ata_port_desc(ap, "cmd 0x%llx ctl 0x%llx",
		(unsigned long long)res->start + PATA_IMX_DRIVE_DATA,
		(unsigned long long)res->start + PATA_IMX_DRIVE_CONTROL);

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	priv->dma_params_rx.dma = res->start;
	priv->dma_params_rx.dma_addr = (unsigned long) priv->host_regs + PATA_IMX_FIFO_DATA;
	dev_dbg(&pdev->dev, "dma %d rx 0x%08lx\n", priv->dma_params_rx.dma, priv->dma_params_rx.dma_addr);

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	priv->dma_params_tx.dma = res->start;
	priv->dma_params_tx.dma_addr = (unsigned long) priv->host_regs + PATA_IMX_FIFO_DATA;
	dev_dbg(&pdev->dev, "dma %d tx 0x%08lx\n", priv->dma_params_tx.dma, priv->dma_params_tx.dma_addr);

	/* deassert resets */
	__raw_writel(PATA_IMX_ATA_CTRL_FIFO_RST_B |
			PATA_IMX_ATA_CTRL_ATA_RST_B,
			priv->host_regs + PATA_IMX_ATA_CONTROL);

	/* enable interrupts */
	__raw_writel(PATA_IMX_ATA_INTR_ATA_INTRQ2,
			priv->host_regs + PATA_IMX_ATA_INT_EN);

	/* activate */
	return ata_host_activate(host, irq, ata_sff_interrupt, 0,
				&pata_imx_sht);

free_priv:
	clk_disable(priv->clk);
	clk_put(priv->clk);
	return -ENOMEM;
}

static int __devexit pata_imx_remove(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct pata_imx_priv *priv = host->private_data;

	ata_host_detach(host);

	__raw_writel(0, priv->host_regs + PATA_IMX_ATA_INT_EN);

	clk_disable(priv->clk);
	clk_put(priv->clk);

	return 0;
}

#ifdef CONFIG_PM
static int pata_imx_suspend(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct pata_imx_priv *priv = host->private_data;
	int ret;

	ret = ata_host_suspend(host, PMSG_SUSPEND);
	if (!ret) {
		__raw_writel(0, priv->host_regs + PATA_IMX_ATA_INT_EN);
		priv->ata_ctl =
			__raw_readl(priv->host_regs + PATA_IMX_ATA_CONTROL);
		clk_disable(priv->clk);
	}

	return ret;
}

static int pata_imx_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct pata_imx_priv *priv = host->private_data;

	clk_enable(priv->clk);

	__raw_writel(priv->ata_ctl, priv->host_regs + PATA_IMX_ATA_CONTROL);

	__raw_writel(PATA_IMX_ATA_INTR_ATA_INTRQ2,
			priv->host_regs + PATA_IMX_ATA_INT_EN);

	ata_host_resume(host);

	return 0;
}

static const struct dev_pm_ops pata_imx_pm_ops = {
	.suspend	= pata_imx_suspend,
	.resume		= pata_imx_resume,
};
#endif

static struct platform_driver pata_imx_driver = {
	.probe		= pata_imx_probe,
	.remove		= __devexit_p(pata_imx_remove),
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM
		.pm		= &pata_imx_pm_ops,
#endif
	},
};

static int __init pata_imx_init(void)
{
	return platform_driver_register(&pata_imx_driver);
}

static void __exit pata_imx_exit(void)
{
	platform_driver_unregister(&pata_imx_driver);
}
module_init(pata_imx_init);
module_exit(pata_imx_exit);

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>");
MODULE_DESCRIPTION("low-level driver for iMX PATA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
