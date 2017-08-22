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
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/output_pwm.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_load.h>



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

bool etat_arm();
bool force_arm();
void affichage_sortie();
void tourne(bool sens);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("ancre : {desc|mont|stop|arm|status}\n\n");
}

void tourne(bool desc)
{

	bool armed = true;
	//pre_armed = true;
	unsigned mixed = 1;
	//mixed = mixer_group.mix(&outputs[0], output_max, nullptr);
	const uint16_t disarmed_pwm[1] = {};
	const uint16_t min_pwm[1] = {PWM_DEFAULT_MIN};
	const uint16_t max_pwm[1] = {PWM_DEFAULT_MAX};
	float output[1]; 
	uint16_t effective_pwm[1] = {1};
	pwm_limit_t _pwm_limit;

	if(desc)
	{
		PX4_INFO("Exec desc");
		output[0] = {1.0f};
	}
	else
	{
		PX4_INFO("Exec mont");
		output[0] = {-1.0f};	
	}

	pwm_limit_init(&_pwm_limit);
	_pwm_limit.state = PWM_LIMIT_STATE_ON;

	pwm_limit_calc(armed,
				 false/*pre_armed*/, mixed /*num_channels*/,
			     0/*const uint16_t reverse_mask*/,
			     disarmed_pwm,
			     min_pwm, 
			     max_pwm,
			     output, 
			     effective_pwm, 
			     &_pwm_limit);
	
	sleep(5);
	output[0] = {0.0f};

	PX4_INFO("Arret");

	pwm_limit_calc(false,
				 false/*pre_armed*/, mixed /*num_channels*/,
			     0/*const uint16_t reverse_mask*/,
			     disarmed_pwm,
			     min_pwm, 
			     max_pwm,
			     output, 
			     effective_pwm, 
			     &_pwm_limit);
}


bool force_arm()//Force arm du drone
{
	struct actuator_armed_s actu;
	memset(&actu, 0, sizeof(actu));
	orb_advert_t actu_pub_fd = orb_advertise(ORB_ID(actuator_armed), &actu);
	actu.armed = true;
	orb_publish(ORB_ID(actuator_armed), actu_pub_fd, &actu);
	if(etat_arm())
		PX4_INFO("Arm reussi");
	else
		PX4_INFO("Arm echoue");
	return(0);
}

bool etat_arm()//Drone armé ou pas
{
	int actuator_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	struct actuator_armed_s etat;
	orb_copy(ORB_ID(actuator_armed), actuator_sub_fd, &etat);
	orb_unsubscribe(actuator_sub_fd);
	return(etat.armed);
}

int ancre_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("Erreur de commande");
        return 1;
    }

    if(!strcmp(argv[1], "arm"))
    {
    	force_arm();
    	return(0);
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
		bool arm = etat_arm();
		if(arm)
			PX4_INFO("Drone arme");
		if(!arm)
			PX4_INFO("Drone non arme");
		return(0);
    }//Fin affichage status


    if(thread_running) 
    {
        warnx("ancre en cours\n");
            if(!strcmp(argv[1], "stop"))
    		{

		    	px4_task_delete(ancre_thread_task);
		    	up_pwm_servo_arm(true);
		    	up_pwm_servo_set(port_ancre, 0);
		    	up_pwm_servo_arm(false);
		    	
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

void affichage_sortie()
{
	int sortie_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
	struct actuator_outputs_s etat_sortie;
	orb_copy(ORB_ID(actuator_outputs), sortie_sub_fd, &etat_sortie);
	PX4_INFO("etat_sortie.output[5] est a %.2f", (double)etat_sortie.output[5]);
	orb_unsubscribe(sortie_sub_fd);
}

int ancre_thread_main(int argc, char *argv[])
{
	//float pulse;
	bool desc;
    thread_running = true;
    int pwm_value;
    int ret;

    PX4_INFO("Demarrage ancre_thread\n");

    if (!strcmp(argv[1], "desc")) 
    {
    //	pulse = 1.0f;
    	PX4_INFO("DESC");
    	desc = true;
    	pwm_value = 2000;
    }
    if (!strcmp(argv[1], "mont"))
    {
    //	pulse = -1.0f;
    	PX4_INFO("MONT");
    	desc = false;
    	pwm_value = 500;
    }
		
	PX4_INFO("Demarrage descente ancre");
	en_desc = true;
	en_haut = false;

	for (int c = 0; c < argc; ++c)
	{
		if(!strcmp(argv[c], "pwm"))
		{
			PX4_INFO("Methode PWM");
			up_pwm_servo_set(port_ancre, pwm_value);
			sleep(5);
			up_pwm_servo_set(port_ancre, 0);
		}
		
		if(!strcmp(argv[c], "pub"))
		{
			PX4_INFO("Methode publication uORB");
			struct actuator_outputs_s sortie;
			memset(&sortie, 0, sizeof(sortie));

			sortie.output[6] = pwm_value;
			orb_advert_t sortie_pub_fd = orb_advertise(ORB_ID(actuator_outputs), &sortie);
			orb_publish(ORB_ID(actuator_outputs), sortie_pub_fd, &sortie);
			affichage_sortie();
			sleep(5);
			sortie.output[6] = pwm_value;
			orb_publish(ORB_ID(actuator_outputs), sortie_pub_fd, &sortie);
		}

		if(!strcmp(argv[c], "out"))
		{
			PX4_INFO("Methode publication output_pwm");
			struct output_pwm_s sortie;
			memset(&sortie, 0, sizeof(sortie));

			sortie.values[port_ancre-1] = pwm_value;
			orb_advert_t sortie_pub_fd = orb_advertise(ORB_ID(output_pwm), &sortie);
			orb_publish(ORB_ID(output_pwm), sortie_pub_fd, &sortie);
			affichage_sortie();
			sleep(5);
			sortie.values[port_ancre-1] = 0;
			orb_publish(ORB_ID(output_pwm), sortie_pub_fd, &sortie);
		}

		if(!strcmp(argv[c], "tourne"))
		{
			tourne(desc);
		}


		if(!strcmp(argv[c], "ioc"))
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
					ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						return 1;
					}
				}
			}//Fin boucle affectation

			
			sleep(5);

			pwm_value = 0;

			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {
					ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						return 1;
					}
				}
			}//Fin boucle affectation
		}// Fin boucle IOC
		
	}//Fin boucle lecture des paramètres

	en_desc = false;
	en_bas = true;


	affichage_sortie();
    thread_running = false;
    return 0;
}