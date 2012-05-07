#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern unsigned int system_modelid;

static int modelid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%08X\n", system_modelid);
	return 0;
}

static int modelid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, modelid_proc_show, NULL);
}

static const struct file_operations modelid_proc_fops = {
	.open		= modelid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_modelid_init(void)
{
	proc_create("modelid", 0, NULL, &modelid_proc_fops);
	return 0;
}
module_init(proc_modelid_init);
