
//--- TIPOS DE DATOS Y DECLARACION DE VARS
typedef struct InputCapture_Signals {
	uint8_t Is_First_Captured;
	uint32_t IC_Value1;
	uint32_t IC_Value2;
	uint32_t Difference;
	float	Frequency;
	uint8_t CalculationOK;
} IC_Sig_t;


//--- FUNCIONES
void set_PWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);


void set_PWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
    HAL_TIM_PWM_Stop(&timer,channel);
    TIM_OC_InitTypeDef sConfigOC;
    timer.Init.Period = period;
    HAL_TIM_PWM_Init(&timer);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&timer,&sConfigOC,channel);

    HAL_TIM_PWM_Start(&timer,channel);
}