/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file derive.c
 * SUWAVE drifting fix
 *
 * @author Paul Asquin <paul.asquin@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <px4_shutdown.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_shutdown.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <logger/logger.h>


static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int derive_thread_task;             /**< Handle of daemon task / thread */

extern "C" __EXPORT int derive_main(int argc, char *argv[]);

int plogger_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("derive : {start|stop|status} [... ]\n\n");
}

int plogger_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("Erreur de commande");
        return 1;
    }

    if (!strcmp(argv[1], "start")) 
    {
        if (thread_running) {
                warnx("derive deja en cours\n");
                /* this is not an error */
                return 0;
        }

        thread_should_exit = false;
        plogger_thread_task = px4_task_spawn_cmd("derive",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         7000,
                         derive_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
             
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!thread_running)
        {
            PX4_INFO("Thread derive pas en cours");
        }
        else
        {
            PX4_INFO("Annulation calculs derive.\nMeurtre du tread");
            px4_task_delete(plogger_thread_task);
            thread_running = false;
        }
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) 
        {
            warnx("\tEn cours\n");

        } else 
        {
            warnx("\tNon demarre\n");
        }

        return 0;
    }

    usage("Commande non reconnue");
    return 1;
}

int derive_thread_main(int argc, char *argv[])
{

    thread_running = true;

    PX4_INFO("Demarrage derive_thread\n");

    if(!thread_should_exit) 
    {
    	/*
        for (int i = 0; i < argc; ++i)
        {
            if (!strcmp(argv[i], "-t"))
            {
                
            }


        }*/    
    }
    
    thread_running = false;
    return 0;
}