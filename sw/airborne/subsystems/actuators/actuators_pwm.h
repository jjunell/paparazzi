/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_PWM_H
#define ACTUATORS_PWM_H

#include "std.h"
#include "generated/airframe.h"

#include BOARD_CONFIG
#include "subsystems/actuators/actuators_pwm_arch.h"

#if USE_SUPERVISION
#include "subsystems/actuators/supervision.h"
#endif

extern void actuators_pwm_init(void);

#define ActuatorsPwmInit() actuators_pwm_init()
/* ActuatorSet and ActuatorsPwmCommit are implemented in arch files
 */

#endif /* ACTUATORS_PWM_H */
