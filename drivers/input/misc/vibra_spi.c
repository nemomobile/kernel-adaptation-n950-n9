/*
 * This file implements a driver for SPI data driven vibrator.
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Ilkka Koskinen <ilkka.koskinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/spi/vibra.h>
#include <linux/io.h>
#include <linux/uaccess.h>

/* Number of effects handled with memoryless devices */
#define WAVE_SIZE		156 /* In u32 */
#define WAIT_TIMEOUT		10 /* In ms */

#define FF_EFFECT_QUEUED	BIT(0)
#define FF_EFFECT_PLAYING	BIT(1)
#define FF_EFFECT_ABORTING	BIT(2)

/* A pre-generated normalized sine-table for signal generation */
static u8 sine_lookup[] = {
	32, 32, 32, 32, 32, 32, 32, 31, 31, 31, 31, 30, 30, 30, 29, 29, 29, 28,
	28, 27, 27, 27, 26, 26, 25, 24, 24, 23, 23, 22, 22, 21, 20, 20, 19, 18,
	18, 17, 16, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7,
	6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7,
	8, 8, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18,
	19, 20, 20, 21, 22, 22, 23, 23, 24, 24, 25, 26, 26, 27, 27, 27, 28, 28,
	29, 29, 29, 30, 30, 30, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32
};

enum vibra_status {
	IDLE = 0,
	STARTED,
	PLAYING,
	CLOSING,
};

struct vibra_effect_info {
	char		*buf;
	unsigned int	buflen;
	unsigned long	flags;	/* effect state (STARTED, PLAYING, etc) */
};

struct vibra_data {
	struct device			*dev;
	struct input_dev		*input_dev;

	struct workqueue_struct 	*workqueue;
	struct work_struct		play_work;

	struct spi_device		*spi_dev;
	struct spi_transfer		t;
	struct spi_message		msg;
	u32				spi_max_speed_hz;

	enum vibra_status		status;

	struct vibra_effect_info	einfo;

	wait_queue_head_t		wq;

	void (*set_power)(bool enable);
};

static int vibra_spi_raw_write_effect(struct vibra_data *vibra)
{
	spi_message_init(&vibra->msg);
	memset(&vibra->t, 0, sizeof(vibra->t));

	vibra->t.tx_buf	= vibra->einfo.buf;
	vibra->t.len	= vibra->einfo.buflen;
	spi_message_add_tail(&vibra->t, &vibra->msg);

	return spi_sync(vibra->spi_dev, &vibra->msg);
}

static void vibra_play_work(struct work_struct *work)
{
	DECLARE_WAITQUEUE(wait, current);
	struct vibra_data *vibra = container_of(work,
						struct vibra_data, play_work);
	struct vibra_effect_info *curr = &vibra->einfo;
	int ret;

	add_wait_queue(&vibra->wq, &wait);
	while (1) {
		if (vibra->status == CLOSING)
			goto switch_off;

		if (curr->flags & FF_EFFECT_ABORTING)
			goto switch_off;

		spin_lock_bh(&vibra->input_dev->event_lock);
		curr->flags |= FF_EFFECT_PLAYING;

		spin_unlock_bh(&vibra->input_dev->event_lock);

		if (vibra->status == STARTED) {
			if (vibra->set_power)
				vibra->set_power(true);

			vibra->status = PLAYING;
		}

		ret = vibra_spi_raw_write_effect(vibra);
		if (ret < 0) {
			dev_err(vibra->dev,
				"Error replaying an effect: %d", ret);
			goto switch_off;
		}

		continue;

switch_off:
		if (vibra->set_power)
			vibra->set_power(false);

		curr->flags &= ~FF_EFFECT_PLAYING;
		vibra->status = IDLE;
		remove_wait_queue(&vibra->wq, &wait);
		return;
	}
}


/*
 * Calculate sample table for vibra signal.
 * Each 32bit sample forms one PWM duty cycle. The vibra is a speaker type vibra
 * that needs 150Hz sine signal to give good vibration force. The wave form is
 * created by doing a amplitude modulation with PWM. The wave form is always
 * the same, but we control the strength by changing the amplitude.
 */
static void vibra_spi_create_signal(struct vibra_data *vibra,
						struct ff_effect *effect)
{
	unsigned int i;
	u16 strong, weak;
	u32 gain, div = 0xffffffff / 32;
	u32 *buf = (u32 *)vibra->einfo.buf;

	strong = effect->u.rumble.strong_magnitude;
	weak = effect->u.rumble.weak_magnitude;

	gain = strong;
	gain = gain << 16;
	gain |= weak;
	gain = gain / 32;

	/* Calculate 32 bit PWM cycles for a sine signal */
	for (i = 0; i < WAVE_SIZE; i++)
		buf[i] = (1 << (sine_lookup[i] * gain / div)) - 1;

}

/*
 * Input/Force feedback guarantees that playback() is called with spinlock held
 * and interrupts off.
 */
static int vibra_spi_playback(struct input_dev *input, void *vibra_data,
						struct ff_effect *ff_effect)
{
	struct vibra_data *vibra = (struct vibra_data *) vibra_data;
	struct vibra_effect_info *einfo = &vibra->einfo;

	if (!vibra->workqueue)
		return -ENODEV;

	if (!ff_effect->u.rumble.strong_magnitude &&
					!ff_effect->u.rumble.weak_magnitude) {
		/* Abort the given effect */
		if (einfo->flags & FF_EFFECT_PLAYING)
			einfo->flags |= FF_EFFECT_ABORTING;

		einfo->flags &= ~FF_EFFECT_QUEUED;
	} else {
		einfo->flags |= FF_EFFECT_QUEUED;
		einfo->flags &= ~FF_EFFECT_ABORTING;
		vibra_spi_create_signal(vibra, ff_effect);

		if (vibra->status == IDLE) {
			vibra->status = STARTED;
			queue_work(vibra->workqueue, &vibra->play_work);
		}

		wake_up_interruptible(&vibra->wq);
	}

	return 0;
}

static int vibra_spi_open(struct input_dev *input)
{
	struct vibra_data *vibra = input_get_drvdata(input);

	vibra->workqueue = create_singlethread_workqueue("vibra");
	if (!vibra->workqueue) {
		dev_err(&input->dev, "couldn't create workqueue\n");
		return -ENOMEM;
	}

	return 0;
}

static void vibra_spi_close(struct input_dev *input)
{
	struct vibra_data *vibra = input_get_drvdata(input);

	vibra->status = CLOSING;

	cancel_work_sync(&vibra->play_work);
	INIT_WORK(&vibra->play_work, vibra_play_work);
	destroy_workqueue(vibra->workqueue);
	vibra->workqueue = NULL;

	vibra->status = IDLE;
}

static int __devinit vibra_spi_probe(struct spi_device *spi)
{
	struct vibra_data *vibra;
	struct vibra_spi_platform_data *pdata;
	int ret = -ENOMEM;

	pdata = spi->dev.platform_data;
	if (!pdata)
		return -ENODEV;

	vibra = kzalloc(sizeof(*vibra), GFP_KERNEL);
	if (!vibra) {
		dev_err(&spi->dev, "Not enough memory");
		return -ENOMEM;
	}

	vibra->spi_max_speed_hz = spi->max_speed_hz;
	vibra->set_power = pdata->set_power;

	INIT_WORK(&vibra->play_work, vibra_play_work);
	init_waitqueue_head(&vibra->wq);

	vibra->dev = &spi->dev;
	spi_set_drvdata(spi, vibra);
	vibra->spi_dev = spi;

	spi->bits_per_word = 32;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi_setup failed");
		goto err_spi_setup;
	}

	vibra->input_dev = input_allocate_device();
	if (!vibra->input_dev) {
		dev_err(vibra->dev, "couldn't allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	input_set_drvdata(vibra->input_dev, vibra);

	vibra->input_dev->name		= "SPI vibrator";
	vibra->input_dev->id.version	= 1;
	vibra->input_dev->dev.parent	= spi->dev.parent;
	vibra->input_dev->open		= vibra_spi_open;
	vibra->input_dev->close		= vibra_spi_close;

	set_bit(FF_RUMBLE, vibra->input_dev->ffbit);

	ret = input_ff_create_memless(vibra->input_dev, vibra,
							vibra_spi_playback);
	if (ret) {
		dev_err(&spi->dev, "Couldn't create ff_memless device");
		goto err_ff_memless_create;
	}

	ret = input_register_device(vibra->input_dev);
	if (ret < 0) {
		dev_dbg(&spi->dev, "couldn't register input device\n");
		goto err_input_register;
	}
	vibra->einfo.buflen = sizeof(u32) * WAVE_SIZE;

	vibra->einfo.buf = kzalloc(vibra->einfo.buflen, GFP_KERNEL);
	if (!vibra->einfo.buf) {
		dev_err(&spi->dev, "Not enough memory for sample table\n");
		goto err_playbuf;
	}

	dev_dbg(&spi->dev, "SPI driven Vibra driver initialized\n");
	return 0;
err_playbuf:
	input_unregister_device(vibra->input_dev);
err_input_register:
	input_ff_destroy(vibra->input_dev);
err_ff_memless_create:
	input_free_device(vibra->input_dev);
err_input_alloc:
err_spi_setup:
	kfree(vibra);
	return ret;
}

static int __devexit vibra_spi_remove(struct spi_device *spi)
{
	struct vibra_data *vibra = dev_get_drvdata(&spi->dev);

	kfree(vibra->einfo.buf);

	/*
	 * No need to do kfree(vibra) since the following calls
	 * input_ff_destroy, which does kfree(ff->private)
	 */
	input_unregister_device(vibra->input_dev);
	kfree(vibra->einfo.buf);
	return 0;
}

static struct spi_driver vibra_spi_driver = {
	.driver = {
		.name		= "vibra_spi",
		.owner		= THIS_MODULE,
	},

	.probe		= vibra_spi_probe,
	.remove		= __devexit_p(vibra_spi_remove),
};

static int __init vibra_spi_init(void)
{
	return spi_register_driver(&vibra_spi_driver);
}
module_init(vibra_spi_init);

static void __exit vibra_spi_exit(void)
{
	spi_unregister_driver(&vibra_spi_driver);
}
module_exit(vibra_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ilkka Koskinen <ilkka.koskinen@nokia.com>");
MODULE_ALIAS("spi:vibra_spi");