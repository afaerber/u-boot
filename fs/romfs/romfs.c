/*
 * (C) Copyright 2013 Andy Green <andy.green@linaro.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * all of these members are big-endian
 */

#include <common.h>
#include <romfs.h>

struct sb {
	unsigned int be_magic1;
	unsigned int be_magic2;
	unsigned int be_size;
	unsigned int be_checksum;
	char name[0];
};

struct inode {
	unsigned int be_next;
	unsigned int be_spec;
	unsigned int be_size;
	unsigned int be_checksum;
	char name[0];
};

/*
 * these magics are already in big-endian and don't need converting
 */
#define BE_ROMFS_MAGIC1 0x6d6f722d
#define BE_ROMFS_MAGIC2 0x2d736631

const void *romfs;

static unsigned int le(const unsigned int *be)
{
	const unsigned char *c = (const unsigned char *)be;
	return (c[0] << 24) | (c[1] << 16) | (c[2] << 8) | c[3];
}

/*
 * rule is pad strings to 16 byte boundary
 */
static int pad(const char *s)
{
	int n = 0;

	while (*s++)
		n++;

	return (n + 0xf) & ~0xf;
}

static const struct inode * lookup(const void *start, const char *filepath)
{
	const struct sb *sb = (struct sb *)romfs;
	const struct inode *inode = (struct inode *)
				(romfs + sizeof(struct sb) + pad(sb->name));
	const struct inode *level = inode;
	const char *p, *n;
	int m;
	const char *target;

	if (start != romfs) {
		inode = start;
		level = start;
	}

	while (inode != romfs) {

		//printf("%s - %x\n", filepath, inode);

		/* match what we can */
		p = filepath;
		n = inode->name;

		//puts("  ");
		//puts(n);
		//puts("\n");

		while (*p && *p != '/' && *n && *p == *n) {
			p++;
			n++;
		}

		/* matched everything */
		if (!*p && !*n) {
			m = le(&inode->be_next) & 7;
			switch (m) {
			case 0: /* hard link */
				return (struct inode *)
					(romfs + (le(&inode->be_spec) & ~0xf));
			case 3: /* symlink */
				target = ((void *)(inode + 1)) +
							       pad(inode->name);
				//puts("Symlink ");
				//puts(target);
				//puts("\n");
				if (*target == '/') {
					/* reinterpret symlink path from / */
					level = (struct inode *)
						(romfs + sizeof(struct sb) +
								 pad(sb->name));
					target++;
				} /* else reinterpret from cwd */
				inode = lookup(level, target);
				//puts("post-recurse ");
				//printf("%x", inode);
				//puts(" ");
				//puts(filepath);
				//puts("\n");
				continue;
			default: /* file of some kind, or dir */
				return inode;
			}
		}
		/* matched dir */
		if (*p == '/' && !*n) {

			m = le(&inode->be_next) & 7;
			switch (m) {
			case 0: /* hard link */
				return (struct inode *)
					(romfs + (le(&inode->be_spec) & ~0xf));
			case 3: /* symlink */
				target = ((void *)(inode + 1)) +
							       pad(inode->name);
				//puts("Symlink ");
				//puts(target);
				//puts("\n");
				if (*target == '/') {
					/* reinterpret symlink path from / */
					level = (struct inode *)
						(romfs + sizeof(struct sb) +
								 pad(sb->name));
					target++;
				} /* else reinterpret from cwd */
				inode = lookup(level, target);
				if (!inode)
					return 0;
				/* resume looking one level deeper */
				inode = (struct inode *)(((void *)(inode + 1)) +
							pad(inode->name));
				while (*filepath != '/' && *filepath)
					filepath++;
				if (!*filepath)
					return 0;
				filepath++;
				//puts("post-recurse ");
				//printf("%x", inode);
				//puts(" ");
				//puts(filepath);
				//puts("\n");
				continue;
			default: /* file of some kind, or dir */
				/* move past the / */
				filepath = p + 1;

				/* resume looking one level deeper */
				inode = (struct inode *)(((void *)(inode + 1)) +
							pad(inode->name));
				break;
			}
			level = inode;
			continue;
		}

		/* not a match, try the next at this level */

		if (!(le(&inode->be_next) & ~0xf))
			/* no more at this level */
			return 0;

		inode = (struct inode *)(romfs + (le(&inode->be_next) & ~0xf));
	}

	return 0;
}


int
romfs_filesystem_read(const char *filepath, void *buf, unsigned long maxsize)
{
	const struct inode *inode = lookup(romfs, filepath);
	unsigned int len;

	if (!inode) {
		//puts("failed to find ");
		//puts(filepath);
		//puts("\n");
		return 0;
	}

	len = le(&inode->be_size);
	if (len > maxsize)
		len = maxsize;
	memcpy(buf, (const char *)inode->name + pad(inode->name), len);

	return len;
}

int romfs_mount(const void *_romfs)
{
	const struct sb *sb = (struct sb *)_romfs;

	romfs = _romfs;

	return (sb->be_magic1 != BE_ROMFS_MAGIC1 ||
			sb->be_magic2 != BE_ROMFS_MAGIC2);
}
