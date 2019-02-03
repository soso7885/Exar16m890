#include <linux/module.h>
#include <linux/slab.h>
#include <linux/serial_core.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>

#include "8250_exar_16m890.h"

extern int serialxr_register_16m890_port(struct uart_8250_port *up);
extern void serialxr_unregister_16m890_port(int line);

struct of_serialxr_info {
	struct clk *clk;
	int type;
	int line;
};
/*
 * Fill a struct uart_port for a given device node
 */
static int of_platform_serialxr_setup(struct platform_device *ofdev,
			int type, struct uart_port *port,
			struct of_serialxr_info *info)
{
	struct resource resource;
	struct device_node *np = ofdev->dev.of_node;
	u32 clk, spd, prop;
	int ret;

	memset(port, 0, sizeof *port);
	if (of_property_read_u32(np, "clock-frequency", &clk)) {

		/* Get clk rate through clk driver if present */
		info->clk = devm_clk_get(&ofdev->dev, NULL);
		if (IS_ERR(info->clk)) {
			dev_warn(&ofdev->dev,
				"clk or clock-frequency not defined\n");
			return PTR_ERR(info->clk);
		}

		ret = clk_prepare_enable(info->clk);
		if (ret < 0)
			return ret;

		clk = clk_get_rate(info->clk);
	}
	/* If current-speed was set, then try not to change it. */
	if (of_property_read_u32(np, "current-speed", &spd) == 0)
		port->custom_divisor = clk / (16 * spd);

	ret = of_address_to_resource(np, 0, &resource);
	if (ret) {
		dev_warn(&ofdev->dev, "invalid address\n");
		goto out;
	}

	spin_lock_init(&port->lock);
	port->mapbase = resource.start;
	port->mapsize = resource_size(&resource);

	/* Check for shifted address mapping */
	if (of_property_read_u32(np, "reg-offset", &prop) == 0)
		port->mapbase += prop;

	/* Check for registers offset within the devices address range */
	if (of_property_read_u32(np, "reg-shift", &prop) == 0)
		port->regshift = prop;

	/* Check for fifo size */
	if (of_property_read_u32(np, "fifo-size", &prop) == 0)
		port->fifosize = prop;

	/* Check for a fixed line number */
	ret = of_alias_get_id(np, "serial");
	if (ret >= 0)
		port->line = ret;

	port->irq = irq_of_parse_and_map(np, 0);
	port->iotype = UPIO_MEM;
	if (of_property_read_u32(np, "reg-io-width", &prop) == 0) {
		switch (prop) {
		case 1:
			port->iotype = UPIO_MEM;
			break;
		case 4:
			port->iotype = of_device_is_big_endian(np) ?
				       UPIO_MEM32BE : UPIO_MEM32;
			break;
		default:
			dev_warn(&ofdev->dev, "unsupported reg-io-width (%d)\n",
				 prop);
			ret = -EINVAL;
			goto out;
		}
	}

	port->type = type;
	port->uartclk = clk;
	port->flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP |
					UPF_FIXED_PORT | UPF_FIXED_TYPE;

	if (of_find_property(np, "no-loopback-test", NULL))
		port->flags |= UPF_SKIP_TEST;

	port->dev = &ofdev->dev;

	return 0;
out:
	if (info->clk)
		clk_disable_unprepare(info->clk);
	return ret;
}

static const struct of_device_id of_platform_serialxr_table[];
static int of_platform_serialxr_probe(struct platform_device *ofdev)
{
	const struct of_device_id *match;
	struct of_serialxr_info *info;
	struct uart_port port;
	int port_type;
	int ret;

	match = of_match_device(of_platform_serialxr_table, &ofdev->dev);
	if (!match)
		return -EINVAL;

	if (of_find_property(ofdev->dev.of_node, "used-by-rtas", NULL))
		return -EBUSY;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	port_type = (unsigned long)match->data;
	ret = of_platform_serialxr_setup(ofdev, port_type, &port, info);
	if (ret)
		goto out;

	switch (port_type) {
	case PORT_XR16M890:
	{
		struct uart_8250_port port8250;
		memset(&port8250, 0, sizeof(port8250));
		port8250.port = port;

		if (port.fifosize)
			port8250.capabilities = UART_CAP_FIFO;

		if (of_property_read_bool(ofdev->dev.of_node,
					"auto-flow-control"))
			port8250.capabilities |= UART_CAP_AFE;

		ret = serialxr_register_16m890_port(&port8250);
		break;
	}
	default:
		/* need to add code for these */
	case PORT_UNKNOWN:
		dev_info(&ofdev->dev, "Unknown serial port found, ignored\n");
		ret = -ENODEV;
		break;
	}
	if (ret < 0)
		goto out;

	pr_info("Exar 16m890 register port ttyAP%d success!\n", ret);

	info->type = port_type;
	info->line = ret;
	platform_set_drvdata(ofdev, info);
	return 0;
out:
	kfree(info);
	irq_dispose_mapping(port.irq);
	return ret;
}

/*
 * Release a line
 */
static int of_platform_serialxr_remove(struct platform_device *ofdev)
{
	struct of_serialxr_info *info = platform_get_drvdata(ofdev);

	switch (info->type) {
	case PORT_XR16M890:
		serialxr_unregister_16m890_port(info->line);
		break;
	default:
		/* need to add code for these */
		break;
	}

	if (info->clk)
		clk_disable_unprepare(info->clk);
	kfree(info);
	return 0;
}

static const struct of_device_id of_platform_serialxr_table[] = {
	{ .compatible = "exar,16m890", .data = (void *)PORT_XR16M890, },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, of_platform_serialxr_table);

static struct platform_driver of_platform_serialxr_driver = {
	.driver = {
		.name = "of_serialxr",
		.of_match_table = of_platform_serialxr_table,
	},
	.probe = of_platform_serialxr_probe,
	.remove = of_platform_serialxr_remove,
};

module_platform_driver(of_platform_serialxr_driver);
