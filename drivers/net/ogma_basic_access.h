/**
 * ogma_basic_access.h
 *
 *  Copyright (c) 2011 - 2013 Fujitsu Semiconductor Limited.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *
 */

#ifndef OGMA_BASIC_ACCESS_H
#define OGMA_BASIC_ACCESS_H

void ogma_set_mac_reg (
    unsigned long addr,
    unsigned long value);

unsigned long ogma_get_mac_reg (
    unsigned long addr);


#endif/* OGMA_BASIC_ACCESS_H */
