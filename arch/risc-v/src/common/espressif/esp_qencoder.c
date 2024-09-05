/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_qencoder.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "soc/gpio_sig_map.h"
#include "riscv_internal.h"
#include "periph_ctrl.h"
#include "soc/pcnt_reg.h"
#include "esp_gpio.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input filter *************************************************************/

#ifdef CONFIG_ESP_PCNT_U0_QE
#  ifndef CONFIG_ESP_PCNT_U0_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U0"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U1_QE
#  ifndef CONFIG_ESP_PCNT_U1_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U1"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U2_QE
#  ifndef CONFIG_ESP_PCNT_U2_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U2"
#  endif
#endif

#ifdef CONFIG_ESP_PCNT_U3_QE
#  ifndef CONFIG_ESP_PCNT_U3_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U3"
#  endif
#endif

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the quadrature
 * encoder
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SENSORS
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  ifdef CONFIG_DEBUG_INFO
#    define qe_dumpgpio(p,m)    esp_dumpgpio(p,m)
#  else
#    define qe_dumpgpio(p,m)
#  endif
#else
#  define qe_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct esp_qeconfig_s
{
  uint8_t   pcntid;        /* PCNT ID {0,1,2,3} */
  uint8_t   ch0_gpio;      /* Channel 0 gpio pin (Edge/Pulse) */
  uint8_t   ch1_gpio;      /* Channel 1 gpio pin (Level/Ctrl) */
  uint32_t  ch0_pulse_sig; /* ch0 pulse signal index */
  uint32_t  ch0_ctrl_sig;  /* ch0 ctrl signal index */
  uint32_t  ch1_pulse_sig; /* ch1 pulse signal index */
  uint32_t  ch1_ctrl_sig;  /* ch1 ctrl signal index */
  uint16_t  filter_thres;  /* Filter threshold for this PCNT Unit */
};

/* NOTE: we are using Quadrature Encoder in X4 mode on ESP PCNT, then
 * instead of using 'pulse_gpio' and 'ctrl_gpio' names, we only use ch0_gpio
 * and ch1_gpio names. It avoid confusion, since the same signal that is used
 * on pin 'pulse' of CH0 is also connected to 'ctrl' pin of the CH1 and
 * 'ctrl' pin of CH0 is also connected on 'pulse' pin of CH1.
 */

/* Overall, RAM-based state structure */

struct esp_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the
   * lower-half callback structure:
   */

  const struct qe_ops_s *ops; /* Lower half callback structure */

  /* ESP driver-specific fields: */

  const struct esp_qeconfig_s *config; /* static configuration */

  bool inuse; /* True: The lower-half driver is in-use */

  volatile int32_t position; /* The current position offset */

  spinlock_t lock; /* Device specific lock. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void esp_dumpregs(struct esp_lowerhalf_s *priv,
                           const char *msg);
#else
#  define esp_dumpregs(priv,msg)
#endif

static struct esp_lowerhalf_s *esp_pcnt2lower(int pcnt);

/* Interrupt handling */

#if 0 /* FIXME: To be implemented */
static int esp_interrupt(int irq, void *context, void *arg);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int esp_setup(struct qe_lowerhalf_s *lower);
static int esp_shutdown(struct qe_lowerhalf_s *lower);
static int esp_position(struct qe_lowerhalf_s *lower,
                          int32_t *pos);
static int esp_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp_reset(struct qe_lowerhalf_s *lower);
static int esp_setindex(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = esp_setup,
  .shutdown  = esp_shutdown,
  .position  = esp_position,
  .setposmax = esp_setposmax,
  .reset     = esp_reset,
  .setindex  = esp_setindex,
  .ioctl     = esp_ioctl,
};

/* Per-pcnt state structures */

#ifdef CONFIG_ESP_PCNT_U0_QE
static const struct esp_qeconfig_s g_pcnt0config =
{
  .pcntid        = 0,
  .ch0_gpio      = CONFIG_ESP_PCNT_U0_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP_PCNT_U0_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN0_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN0_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN0_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN0_IDX,
  .filter_thres  = CONFIG_ESP_PCNT_U0_FILTER_THRES,
};

static struct esp_lowerhalf_s g_pcnt0lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt0config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U1_QE
static const struct esp_qeconfig_s g_pcnt1config =
{
  .pcntid        = 1,
  .ch0_gpio      = CONFIG_ESP_PCNT_U1_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP_PCNT_U1_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN1_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN1_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN1_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN1_IDX,
  .filter_thres  = CONFIG_ESP_PCNT_U1_FILTER_THRES,
};

static struct esp_lowerhalf_s g_pcnt1lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt1config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U2_QE
static const struct esp_qeconfig_s g_pcnt2config =
{
  .pcntid       = 2,
  .ch0_gpio     = CONFIG_ESP_PCNT_U2_CH0_EDGE_PIN,
  .ch1_gpio     = CONFIG_ESP_PCNT_U2_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN2_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN2_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN2_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN2_IDX,
  .filter_thres = CONFIG_ESP_PCNT_U2_FILTER_THRES,
};

static struct esp_lowerhalf_s g_pcnt2lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt2config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP_PCNT_U3_QE
static const struct esp_qeconfig_s g_pcnt3config =
{
  .pcntid        = 3,
  .ch0_gpio      = CONFIG_ESP_PCNT_U3_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP_PCNT_U3_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN3_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN3_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN3_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN3_IDX,
  .filter_thres  = CONFIG_ESP_PCNT_U3_FILTER_THRES,
};

static struct esp_lowerhalf_s g_pcnt3lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt3config,
  .inuse    = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the QENCODER block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void esp_dumpregs(struct esp_lowerhalf_s *priv,
                           const char *msg)
{
  sninfo("%s:\n", msg);
  sninfo("  PCNT_U0_CONF0_REG: %08x PCNT_U1_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(0)),
         getreg32(PCNT_CONF0_U(1)));
  sninfo("  PCNT_U2_CONF0_REG: %08x PCNT_U3_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(2)),
         getreg32(PCNT_CONF0_U(3)));
  sninfo("  PCNT_U0_CONF1_REG: %08x PCNT_U1_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(0)),
         getreg32(PCNT_CONF1_U(1)));
  sninfo("  PCNT_U2_CONF1_REG: %08x PCNT_U3_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(2)),
         getreg32(PCNT_CONF1_U(3)));
  sninfo("  PCNT_U0_CONF2_REG: %08x PCNT_U1_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(0)),
         getreg32(PCNT_CONF2_U(1)));
  sninfo("  PCNT_U2_CONF2_REG: %08x PCNT_U3_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(2)),
         getreg32(PCNT_CONF2_U(3)));
  sninfo("  PCNT_U0_CNT_REG: %08x PCNT_U1_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(0)),
         getreg32(PCNT_CNT_U(1)));
  sninfo("  PCNT_U2_CNT_REG: %08x PCNT_U3_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(2)),
         getreg32(PCNT_CNT_U(3)));
  sninfo("  PCNT_CTRL_REF: %08x\n",
         getreg32(PCNT_CTRL_REG));
}
#endif

/****************************************************************************
 * Name: esp_pcnt2lower
 *
 * Description:
 *   Map a PCNT number to a device structure
 *
 ****************************************************************************/

static struct esp_lowerhalf_s *esp_pcnt2lower(int pcnt)
{
  switch (pcnt)
    {
#ifdef CONFIG_ESP_PCNT_U0_QE
    case 0:
      return &g_pcnt0lower;
#endif
#ifdef CONFIG_ESP_PCNT_U1_QE
    case 1:
      return &g_pcnt1lower;
#endif
#ifdef CONFIG_ESP_PCNT_U2_QE
    case 2:
      return &g_pcnt2lower;
#endif
#ifdef CONFIG_ESP_PCNT_U3_QE
    case 3:
      return &g_pcnt3lower;
#endif
    default:
      return NULL;
    }
}

/****************************************************************************
 * Name: esp_interrupt
 *
 * Description:
 *   Common timer interrupt handling.  NOTE: Only 16-bit timers require timer
 *   interrupts.
 *
 ****************************************************************************/

#if 0 /* FIXME: To be implemented */
static int esp_interrupt(int irq, void *context, void *arg)
{
  struct esp_lowerhalf_s *priv = (struct esp_lowerhalf_s *)arg;
  uint16_t regval;

  DEBUGASSERT(priv != NULL);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *
 *
 ****************************************************************************/

static int esp_setup(struct qe_lowerhalf_s *lower)
{
  struct esp_lowerhalf_s *priv = (struct esp_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t regval;

  /* Protected access to the registers */

  flags = spin_lock_irqsave(&priv->lock);

  esp_dumpregs(priv, "Before setup");

  /* Enable the PCNT Clock and Reset the peripheral */

  periph_module_enable(PERIPH_PCNT_MODULE);
  periph_module_reset(PERIPH_PCNT_MODULE);

  /* Disable all events */

  putreg32(0, PCNT_CONF0_U(priv->config->pcntid));

  /* Configure the limits range PCNT_CNT_L_LIM | PCNT_CNT_H_LIM */

  regval  = INT16_MIN << 16;
  regval |= INT16_MAX;
  putreg32(regval, PCNT_CONF2_U(priv->config->pcntid));

  /* Setup POS/NEG/LCTRL/HCTRL/FILTER modes */

  regval  = priv->config->filter_thres;
  regval |= PCNT_COUNT_INC << PCNT_CH0_NEG_MODE_U0_S;      /* Increase on Falling-Edge */
  regval |= PCNT_COUNT_DEC << PCNT_CH0_POS_MODE_U0_S;      /* Decrease on Rising-Edge */
  regval |= PCNT_MODE_REVERSE << PCNT_CH0_LCTRL_MODE_U0_S; /* Rising A with B in HIGH = CW step */
  regval |= PCNT_MODE_KEEP << PCNT_CH0_HCTRL_MODE_U0_S;    /* Rising A with B in LOW = CCW step */

  putreg32(regval, PCNT_CONF0_U(priv->config->pcntid));

  regval |= PCNT_COUNT_DEC << PCNT_CH1_NEG_MODE_U0_S;      /* Decrease on Falling-Edge */
  regval |= PCNT_COUNT_INC << PCNT_CH1_POS_MODE_U0_S;      /* Increase on Rising-Edge */
  regval |= PCNT_MODE_REVERSE << PCNT_CH1_LCTRL_MODE_U0_S; /* Rising A with B in HIGH = CW step */
  regval |= PCNT_MODE_KEEP << PCNT_CH1_HCTRL_MODE_U0_S;    /* Rising A with B in LOW = CCW step */

  putreg32(regval, PCNT_CONF0_U(priv->config->pcntid));

  /* Configure GPIO pins as Input with Pull-Up enabled */

  esp_configgpio(priv->config->ch0_gpio, INPUT_FUNCTION_3 | PULLUP);
  esp_configgpio(priv->config->ch1_gpio, INPUT_FUNCTION_3 | PULLUP);

  /* Connect Channel A (ch0_gpio) and Channel B (ch1_gpio) crossed for X4 */

  esp_gpio_matrix_in(priv->config->ch0_gpio,
                     priv->config->ch0_pulse_sig, 0);
  esp_gpio_matrix_in(priv->config->ch1_gpio,
                     priv->config->ch0_ctrl_sig, 0);

  esp_gpio_matrix_in(priv->config->ch1_gpio,
                     priv->config->ch1_pulse_sig, 0);
  esp_gpio_matrix_in(priv->config->ch0_gpio,
                     priv->config->ch1_ctrl_sig, 0);

  /* Clear the Reset bit to enable the Pulse Counter */

  regval = getreg32(PCNT_CTRL_REG);
  regval &= ~(1 << (2 * priv->config->pcntid));
  putreg32(regval, PCNT_CTRL_REG);

  esp_dumpregs(priv, "After setup");

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware,
 *   and put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int esp_shutdown(struct qe_lowerhalf_s *lower)
{
  struct esp_lowerhalf_s *priv = (struct esp_lowerhalf_s *)lower;

  /* Disable PCNT clock */

  periph_module_disable(PERIPH_PCNT_MODULE);

  /* Make sure initial position is 0 */

  priv->position = 0;

  return OK;
}

/****************************************************************************
 * Name: esp_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int esp_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct esp_lowerhalf_s *priv = (struct esp_lowerhalf_s *)lower;
  irqstate_t flags;
  int32_t position;
  int16_t count;

  DEBUGASSERT(lower && priv->inuse);

  flags = spin_lock_irqsave(&priv->lock);

  position = priv->position;
  count = (int16_t)(getreg32(PCNT_CNT_U(priv->config->pcntid)) & 0xffff);

  /* Return the position measurement */

  *pos = position + count;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_setposmax
 *
 * Description:
 *   Set the maximum encoder position.
 *
 ****************************************************************************/

static int esp_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ****************************************************************************/

static int esp_reset(struct qe_lowerhalf_s *lower)
{
  struct esp_lowerhalf_s *priv = (struct esp_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t regval;

  sninfo("Resetting position to zero\n");
  DEBUGASSERT(lower && priv->inuse);

  /* Reset this Pulse Counter Unit. */

  flags = spin_lock_irqsave(&priv->lock);

  /* Reset RST bit */

  modifyreg32(PCNT_CTRL_REG, 0, PCNT_CNT_RST_U(priv->config->pcntid));

  /* Clear RST bit to enable counting again */

  modifyreg32(PCNT_CTRL_REG, PCNT_CNT_RST_U(priv->config->pcntid), 0);

  priv->position = 0;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp_setindex
 *
 * Description:
 *   Set the index pin position
 *
 ****************************************************************************/

static int esp_setindex(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int esp_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  /* No ioctl commands supported */

  /* TODO add an IOCTL to control the encoder pulse count prescaler */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   pcnt    - The PCNT number to used.  'pcnt' must be an element of
 *             {0,1,2,3}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int esp_qeinitialize(const char *devpath, int pcnt)
{
  struct esp_lowerhalf_s *priv;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this
   * timer
   */

  priv = esp_pcnt2lower(pcnt);
  if (priv == NULL)
    {
      snerr("ERROR: PCNT%d support not configured\n", pcnt);
      return -ENXIO;
    }

  /* Make sure that it is available */

  if (priv->inuse)
    {
      snerr("ERROR: PCNT%d is in-use\n", pcnt);
      return -EBUSY;
    }

  /* Register the upper-half driver */

  ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the PCNT is in the shutdown state */

  esp_shutdown((struct qe_lowerhalf_s *)priv);

  /* The driver is now in-use */

  priv->inuse = true;

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
