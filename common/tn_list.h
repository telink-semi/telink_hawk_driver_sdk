/********************************************************************************************************
 * @file	tn_list.h
 *
 * @brief	This is the header file for TLSR8232
 *
 * @author	Driver Group
 * @date	May 8, 2018
 *
 * @par     Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#ifndef _TN_LIST_H_
#define _TN_LIST_H_

#define STRING_CONCAT(s1, s2) s1##s2

typedef void ** tn_list_t;

//Define a list
#define LIST_DEF(name) \
        static void *STRING_CONCAT(name,_list) = 0; \
        static tn_list_t name = (tn_list_t)&STRING_CONCAT(name,_list)

//Define a list inside a structure
#define LIST_STRUCT_DEF(name) \
        void *STRING_CONCAT(name,_list); \
        tn_list_t name

//Initialize a list that is a member of a structure
#define LIST_STRUCT_INIT(struct_ptr, name)                              \
    do {                                                                \
        (struct_ptr)->name = &((struct_ptr)->STRING_CONCAT(name,_list));   \
        (struct_ptr)->STRING_CONCAT(name,_list) = 0;                    \
        tn_list_init((struct_ptr)->name);                                   \
    } while(0)

void   tn_list_init(tn_list_t list);
void * tn_list_head(tn_list_t list);
void * tn_list_tail(tn_list_t list);
void * tn_list_pop (tn_list_t list);
void   tn_list_push(tn_list_t list, void *item);

void * tn_list_chop(tn_list_t list);

void   tn_list_add(tn_list_t list, void *item);
void   tn_list_remove(tn_list_t list, void *item);

int    tn_list_length(tn_list_t list);

void   tn_list_copy(tn_list_t dest, tn_list_t src);

void   tn_list_insert(tn_list_t list, void *previtem, void *newitem);

void * tn_list_item_next(void *item);

#endif /* _TN_LIST_H_ */
