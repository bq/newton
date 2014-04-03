/*
 * rk29_board.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  12/28/2011 11:18:28 AM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */


#ifndef __RK29_BOARD_H 
#define __RK29_BOARD_H 

#if defined(CONFIG_MACH_A70HT3N)
#include "./board/rk29_a70ht3n.h"
#elif defined(CONFIG_MACH_A8)
#include "./board/rk29_a8.h"
#elif defined(CONFIG_MACH_A80HTN)
#include "./board/rk29_a80htn.h"
#endif

#endif
