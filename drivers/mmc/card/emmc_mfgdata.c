/*
 * Parser for Endless mfgdata1
 *
 * Copyright 2015 Endless Mobile, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include <asm/unaligned.h>

#define DATA_BLOB_SIZE 4096

static struct device *mfgdata_dev;
static unsigned char *mfgdata_buf = NULL;
static unsigned int pos = 0;

struct mfgdata_entry {
	const char *tag;
	unsigned char *data;
	u16 length;
	bool valid;
	struct device_attribute attr;
};

static int num_entries = 0;
static struct mfgdata_entry *mfgdata_entries;
static bool mfgdata_initialized = false;

static struct mfgdata_entry *find_entry(const char *tag)
{
	int i;
	for (i = 0; i < num_entries; i++) {
		struct mfgdata_entry *entry = &mfgdata_entries[i];
		if (strcmp(tag, entry->tag) == 0)
			return entry;
	}

	return NULL;
}

/* Read manufacturing data. Returns pointer to immutable buffer and its
 * size. */
int endless_mfgdata_read(const char *tag, const unsigned char **data)
{
	struct mfgdata_entry *entry;

	if (!mfgdata_initialized)
		return -EPROBE_DEFER;

	entry = find_entry(tag);
	if (!entry || !entry->valid)
		return -ENOENT;

	*data = entry->data;
	return entry->length;
}

/* sysfs binary read-out of entry */
static ssize_t mfgdata_attr_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct mfgdata_entry *entry;

	entry = container_of(attr, struct mfgdata_entry, attr);
	memcpy(buf, entry->data, entry->length);
	return entry->length;
}

/*
 * Parse a single entry from the raw data buffer.
 *
 * Returns:
 *  1 : parse OK
 *  0 : EOF
 * -1 : parse fail
 */
static int parse_entry(const char **tag, u16 *length,
					   unsigned char **value)
{
	size_t maxsize = DATA_BLOB_SIZE - pos;
	size_t tag_len = strnlen((char *) mfgdata_buf + pos, maxsize);

	if (tag_len == maxsize)
		return -1;

	if (tag_len == 0)
		return 0;

	*tag = (char *) mfgdata_buf + pos;
	pos += tag_len + 1;

	if (pos >= DATA_BLOB_SIZE - 2)
		return -1;

	*length = get_unaligned_le16(mfgdata_buf + pos);
	pos += 2;

	if (pos >= DATA_BLOB_SIZE - *length)
		return -1;

	*value = mfgdata_buf + pos;
	pos += *length;

	return 1;
}

/* Parse and check format identifier */
static bool parse_header(void)
{
	const char *tag;
	u16 length;
	unsigned char *value;

	if (parse_entry(&tag, &length, &value) != 1)
		return false;

	return length == 2 && strcmp(tag, "MFGDATA") == 0 &&
		   value[0] == 0xdd && value[1] == 0xcc;
}

/* Basic validity checks. We avoid exposing invalid entries. */
static void validate_entry(struct mfgdata_entry *entry)
{
	int i;
	const char *tag = entry->tag;

	if (entry->length == 0)
		goto invalid;

	for (i = 0; i < strlen(tag); i++)
		if (islower(tag[i]))
			goto invalid;

	entry->valid = true;
	return;

invalid:
	pr_info("mfgdata: invalid entry %s\n", entry->tag);
	entry->valid = false;
}

/* Parse entries into mfgdata_entries array */
static bool parse(void)
{
	const char *tag;
	u16 length;
	unsigned char *value;
	int r;
	int tmp;
	int i = 0;

	pos = 0;
	if (!parse_header()) {
		pr_info("mfgdata: no header found\n");
		return false;
	}

	/* Pass 1: count number of entries */
	num_entries = 0;
	tmp = pos;
	while ((r = parse_entry(&tag, &length, &value)) == 1)
		num_entries++;

	if (r == -1) {
		pr_info("mfgdata: parse failure at %d\n", pos);
		num_entries = 0;
		return false;
	}

	mfgdata_entries = kcalloc(num_entries, sizeof(struct mfgdata_entry),
				  GFP_KERNEL);
	pos = tmp;

	/* Pass 2: parse entries, create sysfs attributes */
	for (i = 0; i < num_entries; i++) {
		struct mfgdata_entry *entry = &mfgdata_entries[i];
		r = parse_entry(&entry->tag, &entry->length,
				&entry->data);
		if (r != 1) {
			pr_info("mfgdata: parse failure on tag %d\n", i);
			goto err;
		}

		validate_entry(entry);
		if (!entry->valid)
			continue;

		sysfs_attr_init(&entry->attr->attr);
		entry->attr.attr.name = entry->tag;
		entry->attr.attr.mode = 0400;
		entry->attr.show = mfgdata_attr_show;
		if (device_create_file(mfgdata_dev, &entry->attr)) {
			pr_warn("mfgdata: failed creating attr for %s\n", tag);
			goto err;
		}
	}

	return true;

err:
	num_entries = 0;
	kfree(mfgdata_entries);
	mfgdata_entries = NULL;
	return false;
}

static void prepare_mrq(struct mmc_card *card,
			struct mmc_request *mrq, struct scatterlist *sg,
			unsigned sg_len, u32 dev_addr, unsigned blocks,
			unsigned blksz)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1)
		mrq->cmd->opcode = MMC_READ_MULTIPLE_BLOCK;
	else
		mrq->cmd->opcode = MMC_READ_SINGLE_BLOCK;

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

int emmc_mfgdata_parse(struct mmc_card *card, unsigned long capacity)
{
	unsigned int blocks = DATA_BLOB_SIZE >> card->csd.read_blkbits;
	u32 addr = (capacity - 8192) >> card->csd.read_blkbits;
	struct scatterlist sg;
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	sg_init_one(&sg, mfgdata_buf, DATA_BLOB_SIZE);
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
	prepare_mrq(card, &mrq, &sg, 1, addr, blocks,
		    1 << card->csd.read_blkbits);
	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error) {
		pr_err("mfgdata: command error %d\n", cmd.error);
		return cmd.error;
	}

	if (data.error) {
		pr_err("mfgdata: data error %d\n", cmd.error);
		return data.error;
	}

	parse();
	pr_info("mfgdata: read %d entries\n", num_entries);
	mfgdata_initialized = true;
	return 0;
}

static struct class mfgdata_class = {
	.name = "endless_mfgdata",
	.dev_release = (void(*)(struct device *)) kfree,
};

static int __init emmc_mfgdata_init(void)
{
	int ret;

	mfgdata_buf = kzalloc(DATA_BLOB_SIZE, GFP_KERNEL);
	if (!mfgdata_buf)
		return -ENOMEM;

	ret = class_register(&mfgdata_class);
	if (ret)
		goto fail_free_buf;

	mfgdata_dev = kzalloc(sizeof(*mfgdata_dev), GFP_KERNEL);
	if (!mfgdata_dev) {
		ret = -ENOMEM;
		goto fail_class_unregister;
	}

	mfgdata_dev->class = &mfgdata_class;
	dev_set_name(mfgdata_dev, "entries");
	ret = device_register(mfgdata_dev);
	if (ret)
		goto fail_free_dev;

	return 0;

fail_free_dev:
	kfree(mfgdata_dev);
fail_class_unregister:
	class_unregister(&mfgdata_class);
fail_free_buf:
	kfree(mfgdata_buf);

	return ret;
}
arch_initcall(emmc_mfgdata_init);
