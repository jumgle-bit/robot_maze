#ifndef __STM32F103_MIN_H
#define __STM32F103_MIN_H

#include <stdint.h>

#ifndef __IO
#define __IO volatile
#endif

#define PERIPH_BASE           ((uint32_t)0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000)

#define GPIOA_BASE            (APB2PERIPH_BASE + 0x00000800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x00000C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x00001000)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x00000000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x00000000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x00000400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x00003800)

typedef struct
{
    __IO uint32_t CRL;
    __IO uint32_t CRH;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t BRR;
    __IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct
{
    __IO uint32_t EVCR;
    __IO uint32_t MAPR;
    __IO uint32_t EXTICR[4];
    uint32_t RESERVED0;
    __IO uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t APB2RSTR;
    __IO uint32_t APB1RSTR;
    __IO uint32_t AHBENR;
    __IO uint32_t APB2ENR;
    __IO uint32_t APB1ENR;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
} RCC_TypeDef;

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    uint32_t RESERVED0;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    uint32_t RESERVED1;
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
} TIM_TypeDef;

typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t BRR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t GTPR;
} USART_TypeDef;

#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *) GPIOC_BASE)
#define AFIO                  ((AFIO_TypeDef  *) AFIO_BASE)
#define RCC                   ((RCC_TypeDef  *) RCC_BASE)
#define TIM2                  ((TIM_TypeDef  *) TIM2_BASE)
#define TIM3                  ((TIM_TypeDef  *) TIM3_BASE)
#define USART1                ((USART_TypeDef *) USART1_BASE)

/* RCC bits */
#define RCC_APB2ENR_AFIOEN    ((uint32_t)0x00000001)
#define RCC_APB2ENR_IOPAEN    ((uint32_t)0x00000004)
#define RCC_APB2ENR_IOPBEN    ((uint32_t)0x00000008)
#define RCC_APB2ENR_IOPCEN    ((uint32_t)0x00000010)
#define RCC_APB2ENR_USART1EN  ((uint32_t)0x00004000)
#define RCC_APB1ENR_TIM2EN    ((uint32_t)0x00000001)
#define RCC_APB1ENR_TIM3EN    ((uint32_t)0x00000002)

/* TIM bits */
#define TIM_CR1_CEN           ((uint32_t)0x0001)
#define TIM_CR1_ARPE          ((uint32_t)0x0080)
#define TIM_EGR_UG            ((uint32_t)0x0001)
#define TIM_CCER_CC2E         ((uint32_t)0x0010)
#define TIM_CCER_CC3E         ((uint32_t)0x0100)

/* USART bits */
#define USART_SR_TXE           ((uint32_t)0x00000080)
#define USART_SR_TC            ((uint32_t)0x00000040)
#define USART_CR1_UE           ((uint32_t)0x00002000)
#define USART_CR1_TE           ((uint32_t)0x00000008)
#define USART_CR1_RE           ((uint32_t)0x00000004)

/* GPIO pin configuration nibbles for STM32F1 CRL/CRH */
#define GPIO_CNF_INPUT_FLOATING        0x4
#define GPIO_CNF_INPUT_PULL            0x8
#define GPIO_CNF_OUTPUT_PP_2MHZ        0x2
#define GPIO_CNF_OUTPUT_PP_50MHZ       0x3
#define GPIO_CNF_AF_PP_50MHZ           0xB

#endif
