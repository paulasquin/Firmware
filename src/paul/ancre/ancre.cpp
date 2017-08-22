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
 * @file ancre;cpp
 * SUWAVE logger command
 *
 * @author Paul Asquin <paul.asquin@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cstdint>
#include <cstdbool>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/param/param.h>
#include <drivers/drv_pwm_output.h>

//#include <drivers/drv_hrt.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>

static bool thread_running = false;     /**< daemon status flag */
static bool en_haut = false;
static bool en_bas = false;             /**< etat ancre */
static bool en_desc = false;
static bool en_mont = false;
static int ancre_thread_task;             /**< Handle of daemon task / thread */
static unsigned port_ancre = 6;


extern "C" __EXPORT int ancre_main(int argc, char *argv[]);

int ancre_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
int commande(int pwm_value);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("ancre : {desc|mont|stop|arm|status}\n\n");
}

int ancre_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("Erreur de commande");
        return 1;
    }

    if(!strcmp(argv[1], "status"))
    {
		if (en_bas)
		{
			PX4_INFO("Encre en bas");
		}
		if (en_haut)
		{
			PX4_INFO("Encre en haut");
		}
		if (en_mont)
		{
			PX4_INFO("Encre en montee");
		}
		if (en_desc)
		{
			PX4_INFO("Encre en descente");
		}
		return(0);
    }//Fin affichage status
	if(thread_running) 
	    {
        warnx("ancre en cours\n");
            if(!strcmp(argv[1], "stop"))
    		{
		    	commande(0);
		    	PX4_INFO("Moteur stop. Kill du thread");
		    	px4_task_delete(ancre_thread_task);
    		}
    	return(0);
    }//Fin thread déjà en cours

    if(!strcmp(argv[1], "desc")||!strcmp(argv[1], "mont"))
    {
	    ancre_thread_task = px4_task_spawn_cmd("ancre",
	                     SCHED_DEFAULT,
	                     SCHED_PRIORITY_DEFAULT,
	                     7000,
	                     ancre_thread_main,
	                     (argv) ? (char *const *)&argv[1] : (char *const *)NULL);
	                     //(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
	    return 0;
    }

    usage("Commande non reconnue");
    return 1;
}

int commande(int pwm_value)
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	PX4_INFO("Methode publication iOC");
	int fd = px4_open(dev, 0);
	uint32_t set_mask = 0;
	unsigned single_ch = 0;
	int channels = port_ancre;// Peut être en format 46 pour alimenter les ports 4 & 6

	/* get the number of servo channels */
	unsigned servo_count;
	px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
	while ((single_ch = channels % 10)) {

		set_mask |= 1 << (single_ch - 1);
		channels /= 10;
	}

	for (unsigned i = 0; i < servo_count; i++) {//Affectation des valeurs
		if (set_mask & 1 << i) {
			px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
		}
	}//Fin boucle affectation
	return(0);
}

int ancre_thread_main(int argc, char *argv[])
{
    thread_running = true;
    int pwm_value = 0;
    int temps;
    const int temps_desc = 20;
    const int temps_mont = 18;

    PX4_INFO("Demarrage ancre_thread\n");

    if (!strcmp(argv[1], "desc")) 
    {
    	PX4_INFO("DESC");
		en_desc = true;
		en_haut = false;
    	pwm_value = 2000;
    	temps = temps_desc;
    }
    if (!strcmp(argv[1], "mont"))
    {
    	PX4_INFO("MONT");
    	en_mont = true;
    	en_bas = false;
    	pwm_value = 500;
    	temps=temps_mont;
    }

    commande(pwm_value);

	sleep(temps);

	pwm_value = 0;
	commande(pwm_value);

	if(en_desc)
	{
		en_bas = true;
		en_desc = false;
	}
	if (en_mont)
	{
		en_haut = true;
		en_mont = false;
	}

    thread_running = false;
    return 0;
}