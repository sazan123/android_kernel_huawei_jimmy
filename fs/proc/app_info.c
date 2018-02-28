#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/export.h>
#include <misc/app_info.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
static struct kobject *set_appinfo_node_kobject = NULL;

#define APP_NAME_VALUE_SPLIT_CHAR  "*"

#define SENSOR_MAX                 20

struct info_node
{
    char name[APP_INFO_NAME_LENTH];
    char value[APP_INFO_VALUE_LENTH];
    struct list_head entry;
};

struct sensor_node
{
    char name[APP_INFO_NAME_LENTH];
};

struct sensor_node sensor_type[SENSOR_MAX];

static LIST_HEAD(app_info_list);
static DEFINE_SPINLOCK(app_info_list_lock);

/* add app_info_node interface */
/* name and value use * to Separate */
/* buf like LP-Sensor*avago 9930,name will be LP-Sensor,value will be avago 9930 */
static ssize_t hw_set_app_info_node_store(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    char* AppStr = NULL;
    char* orig_AppStr = NULL;
    char AppName[APP_INFO_NAME_LENTH] = {'\0'};
    char AppValue[APP_INFO_VALUE_LENTH] = {'\0'};
    int name_lenth=0,value_lenth=0,buf_len=0,ret=-1;
    int i = 0;
    bool flag = false;
    char* strtok = NULL;

    buf_len = strlen(buf);
    orig_AppStr = AppStr = kmalloc(buf_len+5, GFP_KERNEL);
    if (!AppStr)
    {
        pr_err("%s:kmalloc fail!\n",__func__);
        return -1;
    }

    memset(AppStr, 0, buf_len+5);
    snprintf(AppStr, buf_len + 5 - 1, "%s", buf);
    strtok = strsep(&AppStr, APP_NAME_VALUE_SPLIT_CHAR);
    if (strtok != NULL)
    {
        name_lenth = strlen(strtok);
        memcpy(AppName,strtok,((name_lenth > (APP_INFO_NAME_LENTH-1))?(APP_INFO_NAME_LENTH-1):name_lenth));
    }
    else
    {
        pr_err("%s: buf name Invalid:%s", __func__,buf);
        goto split_fail_exit;
    }
    strtok = strsep(&AppStr, APP_NAME_VALUE_SPLIT_CHAR);
    if(strtok!=NULL)
    {
        value_lenth = strlen(strtok);
        memcpy(AppValue,strtok,((value_lenth > (APP_INFO_VALUE_LENTH-1))?(APP_INFO_VALUE_LENTH-1):value_lenth));
    }
    else
    {
        pr_err("%s: buf value Invalid:%s", __func__,buf);
        goto split_fail_exit;
    }
    for (i = 0; i < SENSOR_MAX; i++)
    {
        if (0 == strncmp(sensor_type[i].name, AppName,sizeof(AppName)))
        {
            flag = true;
            break;
        }
        else if (0 == strncmp(sensor_type[i].name, "",sizeof(AppName)))
        {
            strncpy(sensor_type[i].name, AppName, sizeof(AppName) - 1);
            break;
        }
    }

    if (!flag)
    {
        ret = app_info_set(AppName, AppValue);
        if (ret < 0)
        {
            pr_err("%s: app_info_set fail", __func__);
            goto split_fail_exit;
        }
    }

split_fail_exit:
    if (orig_AppStr)
        kfree(orig_AppStr);
    return size;
}
static struct kobj_attribute sys_set_appinfo_init = {
    .attr = {.name = "set_app_info_node", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
    .show = NULL,
    .store = hw_set_app_info_node_store,
};

static int app_info_node_init(void)
{
    int err = -100;;
    set_appinfo_node_kobject = kobject_create_and_add("set_app_info", NULL);
    if (!set_appinfo_node_kobject)
    {
        pr_err("%s: create set_app_info folder error!", __func__);
        return -1;
    }

    err = sysfs_create_file(set_appinfo_node_kobject, &sys_set_appinfo_init.attr);
    if (err)
    {
        pr_err("%s: init set_appinfo_node_kobject file fail", __func__);
        return -1;
    }

    return 1;
}

int app_info_set(const char * name, const char * value)
{
    struct info_node *new_node = NULL;
    int name_lenth = 0;
    int value_lenth = 0;

    if(WARN_ON(!name || !value))
        return -1;

    name_lenth = strlen(name);
    value_lenth = strlen(value);

    new_node = kzalloc(sizeof(*new_node), GFP_KERNEL);
    if(new_node == NULL)
    {
        return -1;
    }

    memcpy(new_node->name,name,((name_lenth > (APP_INFO_NAME_LENTH-1))?(APP_INFO_NAME_LENTH-1):name_lenth));
    memcpy(new_node->value,value,((value_lenth > (APP_INFO_VALUE_LENTH-1))?(APP_INFO_VALUE_LENTH-1):value_lenth));

    spin_lock(&app_info_list_lock);
    list_add_tail(&new_node->entry,&app_info_list);
    spin_unlock(&app_info_list_lock);

    return 0;
}

EXPORT_SYMBOL(app_info_set);

static int app_info_proc_show(struct seq_file *m, void *v)
{
    struct info_node *node;

    list_for_each_entry(node, &app_info_list, entry)
    {
        seq_printf(m,"%-32s:%32s\n", node->name, node->value);
    }
    return 0;
}

static int app_info_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, app_info_proc_show, NULL);
}

static const struct file_operations app_info_proc_fops =
{
    .open        = app_info_proc_open,
    .read        = seq_read,
    .llseek        = seq_lseek,
    .release    = single_release,
};

static int __init proc_app_info_init(void)
{
    proc_create("app_info", 0, NULL, &app_info_proc_fops);

    app_info_node_init();
    memset(sensor_type, '\0',sizeof(struct sensor_node)*SENSOR_MAX);

    return 0;
}

module_init(proc_app_info_init);

