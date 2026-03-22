#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/of.h>

// 基本参数
#define DEVICE_NAME "servo_pulse"
#define CLASS_NAME  "servo_pulse_class"
#define MAX_SERVOS  6
#define PWM_PERIOD_NS 20000000U  // 20ms 周期

// 默认配置
static const u32 default_min_ns[MAX_SERVOS]  = {400000, 400000, 1000000, 300000, 300000, 1300000};
static const u32 default_max_ns[MAX_SERVOS]  = {2600000, 1900000, 2600000, 2200000, 2600000, 2600000};
static const u32 default_zero_ns[MAX_SERVOS] = {900000, 1200000, 2000000, 900000, 1300000, 1600000};

struct servo_channel {
	struct pwm_device *pwm;
	u32 min_ns;
	u32 max_ns;
	u32 zero_ns;
	u32 duty_ns;
};

struct servo_pulse_dev {
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	struct mutex lock;
	int num_servos;
	struct servo_channel servos[MAX_SERVOS];
};

static struct servo_pulse_dev *g_dev;

static int clamp_and_apply(struct servo_channel *ch, u32 duty_ns)
{
	struct pwm_state state;

	if (!ch->pwm)
		return -ENODEV;

	// 限幅
	if (duty_ns < ch->min_ns)
		duty_ns = ch->min_ns;
	if (duty_ns > ch->max_ns)
		duty_ns = ch->max_ns;

	ch->duty_ns = duty_ns;

	pwm_get_state(ch->pwm, &state);
	state.period = PWM_PERIOD_NS;
	state.duty_cycle = duty_ns;
	state.enabled = true;
	state.polarity = PWM_POLARITY_NORMAL;

	return pwm_apply_state(ch->pwm, &state);
}

static ssize_t servo_write(struct file *file, const char __user *buf,
			   size_t len, loff_t *ppos)
{
	char kbuf[64];
	int id;
	u32 duty;
	int ret;
	struct servo_pulse_dev *dev = g_dev;

	if (!dev)
		return -ENODEV;

	if (len == 0 || len >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, buf, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = sscanf(kbuf, "%d %u", &id, &duty);
	if (ret != 2)
		return -EINVAL;

	/* 统一使用 0-based 通道编号，避免 0/1 指向同一通道 */
	if (id < 0 || id >= dev->num_servos)
		return -EINVAL;

	mutex_lock(&dev->lock);
	ret = clamp_and_apply(&dev->servos[id], duty);
	mutex_unlock(&dev->lock);

	return ret ? ret : len;
}

static struct file_operations servo_fops = {
	.owner = THIS_MODULE,
	.write = servo_write,
};

static int servo_pulse_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct servo_pulse_dev *dev;
	struct device *sys_dev = &pdev->dev;
	u32 tmp[MAX_SERVOS];
	char pwm_name[16];

	dev = devm_kzalloc(sys_dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);

	// 读取 servo-count，可选，默认 6
	if (of_property_read_u32(sys_dev->of_node, "servo-count", &dev->num_servos))
		dev->num_servos = MAX_SERVOS;
	if (dev->num_servos > MAX_SERVOS)
		dev->num_servos = MAX_SERVOS;
	if (dev->num_servos <= 0)
		return -EINVAL;

	// 读取 min/max/zero
	if (!of_property_read_u32_array(sys_dev->of_node, "servo-min-ns",
					tmp, dev->num_servos)) {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].min_ns = tmp[i];
	} else {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].min_ns = default_min_ns[i];
	}

	if (!of_property_read_u32_array(sys_dev->of_node, "servo-max-ns",
					tmp, dev->num_servos)) {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].max_ns = tmp[i];
	} else {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].max_ns = default_max_ns[i];
	}

	if (!of_property_read_u32_array(sys_dev->of_node, "servo-zero-ns",
					tmp, dev->num_servos)) {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].zero_ns = tmp[i];
	} else {
		for (i = 0; i < dev->num_servos; i++)
			dev->servos[i].zero_ns = default_zero_ns[i];
	}

	// 获取 PWM: 依赖设备树 pwms 属性中列出对应通道
	for (i = 0; i < dev->num_servos; i++) {
		struct pwm_state state;

		
		snprintf(pwm_name, sizeof(pwm_name), "servo%d", i);
		dev->servos[i].pwm = devm_pwm_get(sys_dev, pwm_name);
		if (IS_ERR(dev->servos[i].pwm)) {
			ret = PTR_ERR(dev->servos[i].pwm);
			dev_err(sys_dev, "pwm %d (%s) get failed: %d\n", i, pwm_name, ret);
			return ret;
		}

		// 初始化为 zero_ns 但先不使能
		pwm_get_state(dev->servos[i].pwm, &state);
		state.period = PWM_PERIOD_NS;
		state.duty_cycle = dev->servos[i].zero_ns;
		state.enabled = false;
		state.polarity = PWM_POLARITY_NORMAL;

		ret = pwm_apply_state(dev->servos[i].pwm, &state);
		if (ret) {
			dev_err(sys_dev, "pwm %d apply failed: %d\n", i, ret);
			return ret;
		}

		dev->servos[i].duty_ns = dev->servos[i].zero_ns;
	}

	// 注册字符设备
	ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&dev->cdev, &servo_fops);
	dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&dev->cdev, dev->devt, 1);
	if (ret)
		goto err_unreg_region;

	dev->class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(dev->class)) {
		ret = PTR_ERR(dev->class);
		goto err_cdev_del;
	}

	dev->device = device_create(dev->class, NULL, dev->devt, NULL, DEVICE_NAME);
	if (IS_ERR(dev->device)) {
		ret = PTR_ERR(dev->device);
		goto err_class_destroy;
	}

	g_dev = dev;
	platform_set_drvdata(pdev, dev);

	dev_info(sys_dev, "servo pulse driver probed, servos=%d\n", dev->num_servos);
	return 0;

err_class_destroy:
	class_destroy(dev->class);
err_cdev_del:
	cdev_del(&dev->cdev);
err_unreg_region:
	unregister_chrdev_region(dev->devt, 1);
	return ret;
}

static int servo_pulse_remove(struct platform_device *pdev)
{
	int i;
	struct servo_pulse_dev *dev = platform_get_drvdata(pdev);

	if (!dev)
		return 0;

	if (dev->device)
		device_destroy(dev->class, dev->devt);
	if (dev->class)
		class_destroy(dev->class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devt, 1);

	for (i = 0; i < dev->num_servos; i++) {
		if (dev->servos[i].pwm && pwm_is_enabled(dev->servos[i].pwm))
			pwm_disable(dev->servos[i].pwm);
	}

	g_dev = NULL;
	return 0;
}

static const struct of_device_id servo_pulse_of_match[] = {
	{ .compatible = "orangepi,servo-pulse", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, servo_pulse_of_match);

static struct platform_driver servo_pulse_driver = {
	.probe  = servo_pulse_probe,
	.remove = servo_pulse_remove,
	.driver = {
		.name = "servo_pulse",
		.of_match_table = servo_pulse_of_match,
	},
};

static int __init servo_pulse_init(void)
{
	int ret;

	ret = platform_driver_register(&servo_pulse_driver);
	if (ret)
		return ret;

	pr_info("servo_pulse: driver loaded\n");
	return 0;
}

static void __exit servo_pulse_exit(void)
{
	platform_driver_unregister(&servo_pulse_driver);
	pr_info("servo_pulse: driver unloaded\n");
}

module_init(servo_pulse_init);
module_exit(servo_pulse_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sherlock");
MODULE_DESCRIPTION("Multi-servo PWM driver");
