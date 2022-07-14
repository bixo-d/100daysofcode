 /*****************************************************
 ******************************************************
 *
 *           TAREA Control de Voltaje
 *
 *
 * Entrada: .- mensaje desde promedios cada 10min
 *             con iformacion de resistencia, voltaje y corriente
 *          .- mensaje de feedback del movimiento
 *
 * Salida   .-mensaje con orden a ctrl de sec mov
 *
 * Modificada: 10/10/07 para que la tarea sea la misma para V350 y P19
 *             se incluye inicializacion de vector posicion
 *             en la rutina leer posicion de puente
 *             En la rutina regulador se valida Nro de Motores (linea 316)
 * Modificado  19/05/08 se corrige el envio del grupo 2009 para que envie
 *             el status solo cuando este cambia para el caso de puenet inclinado
 * Modificado  08/11/08 se comentan dos isntrucciones en la subrutina calculo_resistencia_ref
 *             para evitar que apague el estatus de arranque
 * Modificado  19/12/08 se inicializa variable bandera_escribir_BD para control de voltaje
 *             con un motor
 * Modificado  10/06/09 Se introduce condicion de pausa por caida de corriente
 *             para orden de movimiento
 * Modificado  09/07/09 Se introduce condicion de pausa por "bajon de corriente"
 *             para orden de movimiento
 ******************************************************
 ******************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <mqueue.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include "/scv350/include/BD102.h"
#include "/scv350/include/def_estruct_mens.h"
#include "/scv350/include/CVM102.h"
#include "/scv350/include/AD100.h"
#include "/scv350/include/SEM102.h"
#include "/scv350/include/def_SEM.h"
#include "/scv350/include/def_errores.h"
#include "/scv350/include/ED102.h"
#include "/scv350/include/CSS_formato_msg.h"

//------	Declaracion de variables GLOBALES ---------------//
//------	Declaracion de colas y mensajes del Sistema Supervisor -------------//
mqd_t								cola_cms100;
T_mens_CSS					mens_salida_ss;
int
	result,
	band_cambio_status_2000,
	band_cambio_status_2009;
//--------------------------	Declaracion de semaforos ----------------------//
sem_t								*ptr_sem20								=	NULL;

/*****************************************************************************/

//            Procedimiento para Calcular valor en una recta

/*****************************************************************************/

float salida_lineal(float y0,
                    float x0,
                    float y1,
                    float x1,
                    float x) {
	float
	pendiente,
	max,
	min,
	y;

	pendiente = (y0-y1)/(x0-x1);
	y				   = (x-x0)*pendiente + y0;
	if (y1 > y0) {
		max = y1;
		min = y0;
	}
	else{
		max = y0;
		min = y1;
	}
	if (y > max)
		y = max;
	if (y < min)
		y = min;

	return y;

}





/*****************************************************************************/

//            Procedimiento para Calcular resistencia de referencia

/*****************************************************************************/
void calculo_resistencia_ref(float *ptr_banda_sup_x_radic,
							 float *ptr_resist_ref_calculada,
							 int   *ptr_band_sum_r_arranque, //4206
							 int   *ptr_band_ext_cuota_mov,
							 int   tiempo_arranque,
							 float resistencia_de_referencia, //4302
							 float fact_calc_lim_sup_resist_adic) {//4337
/**** Declaracion de variables ****/
	int			 error,
				 ind_tiempo,
				 ind_resist,
				 status_arranque,
				 tiempo_para_r_adicional[16];		        /*Grupo 84*/

	float 		 resist_ref_arranque,
				 resistencia_adicional_arranque[16];		  /*Grupo 85*/

/**** Declaracion de variables estaticas ****/
	static float *ptr_SG_RESIST_ARRANQUE_REAL = NULL;
	static int   *ptr_SG_TIEMPO_ARRANQUE_ENT  = NULL;
	static sem_t *ptr_sem					  = NULL;

	*ptr_banda_sup_x_radic = 0.0;
	resist_ref_arranque    = 0.0;

	status_arranque = edo_digital(INDICE_STATUS_ARRANQUE, &error, VERDADERO);
	if (status_arranque) {
		if (*ptr_band_sum_r_arranque) {
			obtener_sem(SEM_GRUPO84, &ptr_sem, &error);

			if (error	==	E_OK) {
				accesar_BD(LEER | ENTERO, 	/*Lee grupo 84 tabla de tiempos*/
						   GRUPO_TIEMPO_RESIST,
						   SG_TIEMPO_ARRANQUE_ENT,
						   0,
						   (SG_TIEMPO_ARRANQUE_No_ELEM_ENT - 1),
						   sizeof(tiempo_para_r_adicional),
						   (void*)tiempo_para_r_adicional,
						   (void*)&ptr_SG_TIEMPO_ARRANQUE_ENT);
		 	 	/*Liberar Semaforo*/
				liberar_sem(SEM_GRUPO84, &ptr_sem, &error);
			}

			obtener_sem(SEM_GRUPO85, &ptr_sem, &error);
			if (error == E_OK) {
				accesar_BD(LEER | REAL, 	/*Lee grupo de resistencia adicional*/
			   			   GRUPO_TIEMPO_RESIST,
						   SG_RESIST_ARRANQUE_REAL,
						   0,
						   (SG_RESIST_ARRANQUE_No_ELEM_REAL - 1),
						   sizeof(resistencia_adicional_arranque),
						   (void*)resistencia_adicional_arranque,
						   (void*)&ptr_SG_RESIST_ARRANQUE_REAL);
				liberar_sem(SEM_GRUPO85, &ptr_sem, &error);
			}
			ind_tiempo = 0;
			ind_resist = -1;
			while(ind_tiempo < SG_TIEMPO_ARRANQUE_No_ELEM_ENT) {
				if (tiempo_para_r_adicional[ind_tiempo] >= tiempo_arranque) {
					ind_resist = ind_tiempo - 1;
					resist_ref_arranque = resistencia_adicional_arranque[ind_resist];
					status_arranque = VERDADERO;
					ind_tiempo = SG_TIEMPO_ARRANQUE_No_ELEM_ENT - 1;
				}
				ind_tiempo++;
			}
			if (ind_resist ==-1) {
				resist_ref_arranque = resistencia_adicional_arranque[SG_TIEMPO_ARRANQUE_No_ELEM_ENT-1];
				status_arranque = FALSO;
			}
		}
		//		else         //comentado 08/11/08
		//			status_arranque = FALSO;
		if (!status_arranque) {
			*ptr_band_sum_r_arranque = FALSO;
			if (edo_digital(INDICE_STATUS_ARRANQUE, &error, VERDADERO)) {
				cambiar_edo_digital(INDICE_STATUS_ARRANQUE, 0, &error, VERDADERO); // apaga bit de arranque
				band_cambio_status_2000 = VERDADERO;
			}
		}
	}
//	*ptr_band_ext_cuota_mov		 = status_arranque;  //comentado 08/11/08
	*ptr_resist_ref_calculada = resistencia_de_referencia + resist_ref_arranque;
	if (resist_ref_arranque > 0.0)
		*ptr_banda_sup_x_radic = fact_calc_lim_sup_resist_adic;
	return;
}

/*****************************************************************************/
//            Procedimiento para leer posicion de puente
/*****************************************************************************/
void leer_posicion_puente_mm(float *posicion_puente) {
/**** Declaracion de variables ****/
	int
		i,
		error,
		anodo_bd_param_ent_var[SG_PARAM_AN_No_ELEM_ENT], 		  /*Grupo 40*/
    *ptr_velocidad_motor							= &anodo_bd_param_ent_var[22];
	float
		velocidad_puente;

/**** Declaracion de variables estaticas ****/
	static float
		*ptr_SG_PARAM_MOT_REAL	= NULL;
	static int
		*ptr_SG_PARAM_AN_ENT  	= NULL;
	static
		sem_t	*ptr_sem					=	NULL;

	/*Tomar Semaforo Grupo 40*/
	obtener_sem(SEM_GRUPO40, &ptr_sem, &error);
  if (error	==	E_OK) {
		accesar_BD(LEER | ENTERO, 	/*Lee grupo 40*/
		    	 	   GRUPO_ANODOS,
				  		 SG_PARAM_AN_ENT,
				   		 0,
			   			 (SG_PARAM_AN_No_ELEM_ENT-1),
			   			 sizeof(anodo_bd_param_ent_var),
			   			 (void*)anodo_bd_param_ent_var,
			   			 (void*)&ptr_SG_PARAM_AN_ENT);
		 	 /*Liberar Semaforo*/
		liberar_sem(SEM_GRUPO40, &ptr_sem, &error);
  }
		obtener_sem(SEM_GRUPO41, &ptr_sem, &error);
  	if (error	==	E_OK) {
			accesar_BD(LEER | REAL, 	/*Lee grupo 41*/
		    		 	   GRUPO_ANODOS,
					  		 SG_PARAM_MOT_REAL,
				 	  		 0,
			   				 0,
			   				sizeof(velocidad_puente),
			   				 (void*)&velocidad_puente,
			   				 (void*)&ptr_SG_PARAM_MOT_REAL);

			liberar_sem(SEM_GRUPO41, &ptr_sem, &error);
		}
		//memset(&posicion_puente, 0.0, sizeof(posicion_puente));
		posicion_puente[2]	=	0.0;
		for(i=0;i<No_MOTORES;i++) {
			posicion_puente[i]	=	(velocidad_puente/ *ptr_velocidad_motor)*
														((float)anodo_bd_param_ent_var[10+i]);
			posicion_puente[2]	+= posicion_puente[i];
		}
		posicion_puente[2] = posicion_puente[2]/No_MOTORES;

	return;
}
/*****************************************************************************/

//            filtro analogico

/*****************************************************************************/

float filtro_analogico(filtro *datos_filtro,
												float valor_real,
                        float periodo,
                        float tiempo_filtro) {

	float
		factor_filtro;


   if (tiempo_filtro > 1E-16)
      factor_filtro = exp(- periodo/tiempo_filtro);
   else
      factor_filtro = 0.0;

   if (datos_filtro->modo > 1)
     datos_filtro->modo	 = 0;

   switch(datos_filtro->modo) {
			case 0:{
        datos_filtro->valor_posterior = valor_real;
        datos_filtro->modo					   = 1;
        break;
      }
      case 1:{
         factor_filtro                = 1.0 - factor_filtro;
         datos_filtro->valor_posterior = datos_filtro->valor_anterior + factor_filtro *(valor_real - datos_filtro->valor_anterior);
         break;
      }
   }
   if (fabsf(datos_filtro->valor_posterior) < 1E-16)
     datos_filtro->valor_posterior = 0.0;

   datos_filtro->valor_anterior = datos_filtro->valor_posterior;

   return datos_filtro->valor_posterior;

}
/***************************************************
*Subrutina que calcula orden de regulacion
*
*
*****************************************************/
void regulador(float *orden,
							 float cambio_R_mm,
							 float ganancia_BD,
							 float resistencia_referencia_calculada,
							 float resistencia,
							 float factor_amortiguamiento,
							 float min_orden,
							 float max_orden,
							 int *bandera_escribir_BD) {
float
	P_gain;

	P_gain = ganancia_BD /cambio_R_mm;
  orden[0] = P_gain * ( resistencia_referencia_calculada - resistencia);

	if (orden[0] < 0.0)
    orden[0] = orden[0] * factor_amortiguamiento;
 	if (fabsf(orden[0]) < min_orden) {
 		*bandera_escribir_BD = VERDADERO;
 		orden[0] = orden[1] = 0; //reseta orden cuando no se envia por ser minima

 	}
 	else{
    if (orden[0] > max_orden)
    	orden[0] = max_orden;
    else{
    	if (orden[0] < -max_orden)
    		orden[0] = -max_orden;
    }
    if (No_MOTORES==1)
    	orden[1]= 0.0;
    else
			orden[1]=orden[0];
 	}
 	return;
}

/***************************************************
*Subrutina que calcula orden de Nivelacion
*
*
*****************************************************/
void Nivelar_Puente(float *orden,
										float dif_posicion_motores,
							 			int *bandera_escribir_BD,
							 			float min_inclinacion_permitida,
							 			float acum_dif_posicion,
							 			float max_ajuste_acum_dia) {
int
	error;

	if (fabsf(dif_posicion_motores) <= min_inclinacion_permitida) {
		*bandera_escribir_BD = VERDADERO;
	}
	else{
		if (acum_dif_posicion >= max_ajuste_acum_dia) {
			cambiar_edo_digital(INDICE_MOV_MAX_DESEQ_DIA, 1, &error, VERDADERO); //43 2009 bit 4
			band_cambio_status_2009 = VERDADERO;
			//envia MSG Error
			formatear_mensaje(MSG_ERROR, MENS_ENTERO, ID_CTRL_VOLT, E_MAX_MOV_DESNIVEL_12H,
	 										  0, 2, 0, NULL, NULL, &mens_salida_ss);
			result = mq_send(cola_cms100,
											(char *)&mens_salida_ss,
	 										sizeof(mens_salida_ss),
	 										NULL);
			*bandera_escribir_BD = VERDADERO;
		}
		else{
		//Desequilibrio entre 1.5 y 7mm
			if (dif_posicion_motores < 0) { // dif negativa orden mover M1 1mm y M2 0
	   		orden[0] = ORDEN_X_DESEQUILIBRIO;
	   		orden[1] = 0.0;
			}
			else{
	   		orden[1] = ORDEN_X_DESEQUILIBRIO;
	   		orden[0] = 0.0;
			}
		}
	}
	return;
}

/**************************************************
***************************************************
*
*							     PROGRAM PPAL
***************************************************
***************************************************/

int main(int argc, char *argv[]) {


int
	band_prio_ctrl_volt,
	status_mov_ctrl_volt,
	tipo_msg_entrada = 0,
  band_inic_resit_ref,
 	band_inic_resit_ref_ant,
	nro_muestras_resist,
	nro_falla_cambio_r_mm,
	reg_anodos_habilitado,
	band_calc_cambio_r_mm,
	band_ext_cuota_mov,
	band_ext_cuota,
	i = 0,
	status_alg_ctrl,
	contador_tracking,
	band_inic_cuota_mov = FALSO,
  status_recepcion,
  bandera_escribir_BD,
  prioridad_orden_mov = PRIOR_MOV_CTRL_VOLT_NORMAL,
  status_envio,
  status_digital,
  prioridad_ult_mov,
  elemento_fin,
  codigo_error,
  elemento_ini                = 0,
  *ptr_SG_REG_AD_ENT			 		= NULL,
  *ptr_SG_PARAM_SIST_ENT		 	= NULL,
  *ptr_SG_PARAM_AN_ENT				= NULL, /*GRUPO 40*/
  *ptr_SG_REG_EDOS_DIG_ENT		= NULL; /*GRUPO 20*/

float
  factor_amortig,
  amortig,
  mag_amortig,
  mag_amortig_ini = 0,
	prom_mov_reg,
	dif_entre_motores,
	resistencia,
	resist_por_mm_reg,
	cte_t_filt_resist_mm,
	resist_acum,
	banda_sup_x_radic,
	banda_sup_calc_b1,
	resist_ref_sist,
	max_cuota_mov_reg,
	b1_promedio,
	periodo_muestreo,
	corriente,
	resist_medida,
	mov_reg [2] = {0.0, 0.0},
	posicion_puente[3]= {0.0, 0.0, 0.0},
	info,
	*ptr_SG_REG_AD_REAL			 		= NULL,
	*ptr_SG_REG_MED_REAL     		= NULL;


/* EStructura para el filtro **/

	struct {
		float  corta,
           larga;
  }resist_filtrada;

/***    Declaracion de status ***/
int
  status_ctrl_pte,
  falla_motor,
  puente_bloqueado,
  efecto_anodico,
  operac_traseg,
  falla_corriente,
  falla_corr_momentanea,
  status_recup_mov,
  status_equilibrando_puente,
  band_inic_cuota_mov_sem,
  band_dif_resist,
  band_tiempo_ea_min,
  status_subida_puente,
  error_sem,
  tiempo_desde_caida_corr = 0,
  tiempo_desde_bajon_corr = 0,
  edo_topes_ult_mov;

/***                 Grupo 42               ***/
	int
		*ptr_SG_PARAM_AN_ENT_2 			 				= NULL, /*grupo 42*/
		param_ctrl_volt_ent[SG_PARAM_AN_No_ELEM_ENT2],
		*ptr_tiempo_estab_luego_mov			= &param_ctrl_volt_ent[0],
    *ptr_t_max_error_resist_mm      = &param_ctrl_volt_ent[1],
 		*ptr_band_envio_inf_ss    			= &param_ctrl_volt_ent[3],
 		*ptr_band_sum_r_arranque				= &param_ctrl_volt_ent[6],
    *ptr_t_min_espera_luego_ea  		= &param_ctrl_volt_ent[11],
    *ptr_t_min_espera_luego_arranq 	= &param_ctrl_volt_ent[12],
    *ptr_t_min_espera_luego_tras   	= &param_ctrl_volt_ent[13],
    *ptr_nro_max_falla_resist_mm		= &param_ctrl_volt_ent[14],
    *ptr_max_t_falla_resist_mm     	= &param_ctrl_volt_ent[15];

/***                 Grupo 43               ***/
	int
		*ptr_SG_REG_AN_REAL			 = NULL; /*grupo 43*/
	float
		param_ctrl_volt_real[SG_REG_AN_REAL_No_ELEM],
    *ptr_resist_ref                     = &param_ctrl_volt_real[2],
    *ptr_banda_inf_fija                 = &param_ctrl_volt_real[3],
    *ptr_banda_sup_fija                 = &param_ctrl_volt_real[4],
    *ptr_resist_ref_calculada          	= &param_ctrl_volt_real[5],
    *ptr_banda_inf_ctrl					        = &param_ctrl_volt_real[6],
    *ptr_banda_sup_ctrl                 = &param_ctrl_volt_real[7],
    *ptr_maxima_variacion_resis         = &param_ctrl_volt_real[8],
    *ptr_lim_min_cambio_r_mm            = &param_ctrl_volt_real[9],
    *ptr_cambio_resist_mm_teor          = &param_ctrl_volt_real[10],
    *ptr_cambio_resist_mm_calc          = &param_ctrl_volt_real[11],
		*ptr_lim_banda_adapt_1              = &param_ctrl_volt_real[13],
    *ptr_lim_banda_adapt_2              = &param_ctrl_volt_real[14],
    *ptr_lim_banda_adapt_b1             = &param_ctrl_volt_real[15],
    *ptr_resist_adic_track              = &param_ctrl_volt_real[16],
    *ptr_factor_adap_banda_inf          = &param_ctrl_volt_real[17],
    *ptr_ganancia_amortig               = &param_ctrl_volt_real[18],
    *ptr_tiempo_amortig                 = &param_ctrl_volt_real[19],
    *ptr_cte_t_filtro_resis_mm          = &param_ctrl_volt_real[20],
    *ptr_mov_anodo_acum_auto            = &param_ctrl_volt_real[21],
    *ptr_mov_anodo_acum_auto_abajo      = &param_ctrl_volt_real[22],
    *ptr_mov_anod_acum_reg              = &param_ctrl_volt_real[23],
    *ptr_mov_anod_acum_reg_abajo        = &param_ctrl_volt_real[24],
    *ptr_mov_anod_acum_equil            = &param_ctrl_volt_real[25],
    *ptr_max_cuota_mov_12hrs            = &param_ctrl_volt_real[26],
    *ptr_max_cuota_mov_equil_12hrs      = &param_ctrl_volt_real[27],
    *ptr_orden_mov_anodo  						  = &param_ctrl_volt_real[29],
    *ptr_lim_inclinacion_pte            = &param_ctrl_volt_real[31],
    *ptr_max_inclinacion_falla          = &param_ctrl_volt_real[32],
    *ptr_max_dif_reg_orden              = &param_ctrl_volt_real[33],
    *ptr_min_mov_ordenado               = &param_ctrl_volt_real[34],
    *ptr_max_mov_ordenado               = &param_ctrl_volt_real[35],
    *ptr_min_resist                     = &param_ctrl_volt_real[36],
    *ptr_banda_adic_con_resit_adic     	= &param_ctrl_volt_real[37],
    *ptr_min_tamano_amortig             = &param_ctrl_volt_real[39],
    *ptr_max_tamano_amortig             = &param_ctrl_volt_real[40],
    *ptr_factor_min_amortig             = &param_ctrl_volt_real[41],
    *ptr_factor_max_amortig             = &param_ctrl_volt_real[42],
    *ptr_ganancia_regular_mov     			= &param_ctrl_volt_real[43],
    *ptr_incr_cuota_arranque            = &param_ctrl_volt_real[44],
    *ptr_lim_banda_adapt_b1_X0          = &param_ctrl_volt_real[45];

/***                   GRUPO 21              ***/
	int
		*ptr_SG_REG_CONT_DIG_ENT 			 = NULL; /*grupo 21*/
	int
 		digital_teller_word_var [SG_REG_CONT_DIG_No_ELEM_ENT],
 		*ptr_tiempo_desde_ea                = &digital_teller_word_var[4],
		*ptr_tiempo_desde_traseg            = &digital_teller_word_var[5],
		*ptr_tiempo_desde_caida_corr        = &digital_teller_word_var[8],
		*ptr_tiempo_desde_bajon_corr        = &digital_teller_word_var[9],
		*ptr_tiempo_anodo_en_manual         = &digital_teller_word_var[13],
		*ptr_tiempo_desde_arranque          = &digital_teller_word_var[16],
		*ptr_tiempo_desde_ult_ord_ctrl      = &digital_teller_word_var[17];

		/***                   GRUPO 10             ***/
	int
		*ptr_SG_PARAM_MED_ENT 			 = NULL; /*grupo 10*/
	int
 		grupo_mediciones [SG_PARAM_MED_No_ELEM_ENT],
 		*ptr_tiempo_calc_prom_resist_mov        = &grupo_mediciones[1],
		*ptr_intervalo_activ_ctrl_volt          = &grupo_mediciones[7];

//------------------	Declaracion de filtros  ------------//
	filtro 						filtro_mfa;

//------------------	Declaracion de colas y mensajes de las colas  ------------//
	struct mq_attr 			  mqstat, *ptr_mqstat =&mqstat;
  T_union_mens		  		msg_ctrl_voltaje,
  										  msg_ctrl_sec_mov,
									    	msg_tiempo_real;
  mqd_t									cola_ctrl_secuencia_mov,
  											cola_ctrl_voltaje,
  											cola_reloj_tiempo_real;


//------------------------  Variables manejo de semaforo-------------------------//
	sem_t	*ptr_sem	=	NULL;
	int error;
//------------------	Declaracion de reloj y buffer para escribirlo  ------------//
	time_t time_of_day;
  char buffer[ 80 ];

/*Inicializacion*/

// Inicio de colas

	ptr_mqstat->mq_maxmsg = 10;
	ptr_mqstat->mq_msgsize= sizeof(T_union_mens);
 	cola_ctrl_secuencia_mov = mq_open(COLA_CSM100,
								            	    	O_WRONLY |O_CREAT|O_NONBLOCK,
             												0xFFFF,
								               			ptr_mqstat);
 	cola_ctrl_voltaje 		 = mq_open(COLA_CV100,
			           	    	 					 O_RDONLY |O_CREAT,
      		         			 					 0xFFFF,
								               		 ptr_mqstat);
	//	Cola del reloj en tiempo real
  cola_reloj_tiempo_real	= mq_open(COLA_REL100,
																		O_WRONLY |O_CREAT |O_NONBLOCK,
																		0xFFFF,
																		ptr_mqstat);
	// Cola msg Sistema supervisor
	ptr_mqstat->mq_maxmsg = 50;
	cola_cms100 						= mq_open(COLA_CMS100,
																		O_WRONLY |O_CREAT |O_NONBLOCK,
																		0xFFFF,
																		ptr_mqstat);
  cambiar_edo_digital(INDICE_SEMAFORO_TRASEG, 0, &error, VERDADERO);
	obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
	if (error	==	E_OK) {
		accesar_BD(LEER | REAL, 	/*Lee grupo43 */
			     	  GRUPO_ANODOS,
					    SG_REG_AN_REAL	,
					    10,
					    10,
				  	  sizeof(*ptr_cambio_resist_mm_teor),
				    	(void*)ptr_cambio_resist_mm_teor,
				      (void*)&ptr_SG_REG_AN_REAL);
  	liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
	}
  mag_amortig         = 1.0;
  resist_por_mm_reg  = *ptr_cambio_resist_mm_teor;
  filtro_mfa.modo = 0;
  *ptr_cambio_resist_mm_calc = filtro_analogico(&filtro_mfa,
													                      resist_por_mm_reg,
                        	                      1.0,
                        	                      1.0);

  nro_falla_cambio_r_mm = 0;

// programa TIMER para despertar cada 12 Horas
  memset(&msg_tiempo_real, 0, sizeof(msg_tiempo_real));
	strcpy(msg_tiempo_real.mens_rel.cola_resp, COLA_CV100);
	msg_tiempo_real.mens_rel.tipo = MENS_NUEVO;
	msg_tiempo_real.mens_rel.tiempo_temporizado = 720;
	msg_tiempo_real.mens_rel.tiempo_rearme = 720;
	msg_tiempo_real.mens_rel.base_tiempo = MIN;
	msg_tiempo_real.mens_rel.tipo_temporizado = 1;
	status_envio = mq_send(cola_reloj_tiempo_real,
							         (char *)&msg_tiempo_real,
           						 sizeof(msg_tiempo_real),
           	  	  		 0);
  time_of_day = time( NULL );
  strftime( buffer, 80, "Hora Arranque : %T", localtime( &time_of_day ) );
	band_ext_cuota 				  = FALSO;
	band_inic_resit_ref		  = FALSO;
  band_inic_resit_ref_ant = VERDADERO;
	resist_ref_sist         = 0.0;
	resistencia							= 0.0;
 	periodo_muestreo        = 10.0;
 	status_mov_ctrl_volt		= 0;
// Envio msg 2009 al supervisor
	obtener_sem(SEM_GRUPO20, &ptr_sem, &error);
	if (error	==	E_OK) {
		accesar_BD(LEER | ENTERO, 	/*Lee BD 2009*/
	  			 	   GRUPO_DIGITAL,
	 						 SG_REG_EDOS_DIG_ENT,
							 9,
							 9,
							 sizeof(status_digital),
							 (void*)&status_digital,
							 (void*)&ptr_SG_REG_EDOS_DIG_ENT);
		liberar_sem(SEM_GRUPO20, &ptr_sem, &error);
	}
	formatear_mensaje(MSG_IENT,
    								MENS_ENTERO,
      							GRUPO_DIGITAL,
      							SG_REG_EDOS_DIG_ENT,
      							9,
      							4,
      							0,
      							&status_digital,
      							NULL,
      							&mens_salida_ss);
   result = mq_send(cola_cms100,
      	     				(char *)&mens_salida_ss,
       							 sizeof(mens_salida_ss),
       							 NULL);
/* fin de bloque Inicializacion	*/

	while(VERDADERO) { // repita por siempre E_CTRL_PTE_MAN_1H
// Espera msg de la cola de ordenes
  	status_recepcion= mq_receive (cola_ctrl_voltaje,
          					   					 (char *)&msg_ctrl_voltaje,
					             					  sizeof(msg_ctrl_voltaje),
				               					  NULL);
// OPto LEE teimpo desde ultima orden de Ctrl Voltaje
// OPto LEE grupo 41 velocidad de motor;

		if (status_recepcion >0) {
      switch (msg_ctrl_voltaje.mens_cv.tipo) {
      	case MENS_REALIM:{ //msg desde ctrl de sec de motores
      		for(i=0;i<No_MOTORES;i++)
      			mov_reg[i]  				=  msg_ctrl_voltaje.mens_cv.mov_reg_mm[i];
      		status_mov_ctrl_volt 	=  msg_ctrl_voltaje.mens_cv.estado;
          tipo_msg_entrada    	=  1;
				break;
      	}
        case MENS_NUEVO:{ //msg desde promedios analogicos
          periodo_muestreo      = msg_ctrl_voltaje.mens_cv.periodo;
          corriente             = msg_ctrl_voltaje.mens_cv.corriente;
          resist_medida         = msg_ctrl_voltaje.mens_cv.resistencia;
          resist_filtrada.corta = msg_ctrl_voltaje.mens_cv.resist_corta;
          resist_filtrada.larga = msg_ctrl_voltaje.mens_cv.resist_larga;
          tipo_msg_entrada      = 2;
        break;
        }
				case MENS_TIEMPO_EXPIRO:{
					// msg de reloj en tiempo real cada 12 hrs
					// escribe hora en pantalla
    			time_of_day = time( NULL );
    			strftime( buffer, 80, "Hora: %T", localtime( &time_of_day ) );
					band_inic_cuota_mov = VERDADERO;
        	tipo_msg_entrada     = 0;
        break;
				}
      } //fin del switch

      switch(tipo_msg_entrada) {
        case 0:{

        break;
        }
        case 1:{   //retorno luego de movimiento
        	prom_mov_reg  = 0.0;
        	for(i=0;i<No_MOTORES;i++)
        		prom_mov_reg = prom_mov_reg + mov_reg[i];
        	prom_mov_reg = prom_mov_reg / No_MOTORES;
          *ptr_mov_anod_acum_reg = *ptr_mov_anod_acum_reg + fabsf(prom_mov_reg); //mov acumulado
          if (prom_mov_reg < 0.0) {
             *ptr_mov_anod_acum_reg_abajo = *ptr_mov_anod_acum_reg_abajo + fabsf(prom_mov_reg);
          }
          obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
					if (error	==	E_OK) {
						accesar_BD(ESCRIBIR | REAL, 	//Escribe grupo 43
		     	  						GRUPO_ANODOS,
				    						SG_REG_AN_REAL	,
				    						23,
				    						24,
			  	  						(sizeof(*ptr_mov_anod_acum_reg)*2),
			    							(void*)&param_ctrl_volt_real[23],
			      						(void*)&ptr_SG_REG_AN_REAL);
				  	liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
					}
          if (status_mov_ctrl_volt == NORMAL) {
          	// si mov sin error chequea si dif  entre registrado y ordenado E_CTRL_PTE_MAN_1H
          	// mayor a permitido
          	for(i=0;i<No_MOTORES;i++) {
            	if ((fabsf(mov_reg[i] - ptr_orden_mov_anodo[i]) > *ptr_max_dif_reg_orden))
         		 		status_mov_ctrl_volt = E_MOV_REG_MAY_ORD;
          	}
         	}
          if (status_mov_ctrl_volt != NORMAL) {
//            Enviar msg de error
						formatear_mensaje(MSG_ERROR,
  								 					  MENS_ENTERO,
					      							ID_CTRL_VOLT,
    													status_mov_ctrl_volt,
    							 						0,
    							 						2,
    							 						0,
					      						  NULL,
					      						  NULL,
    													&mens_salida_ss);
					  result = mq_send(cola_cms100,
    	     									(char *)&mens_salida_ss,
     							 					sizeof(mens_salida_ss),
     							 					NULL);
//            Cambiar status digital de dif maxima entre ordenado y registrado 2009 bit 5
						cambiar_edo_digital(INDICE_DIF_MAX_ORDEN_MENOS_REG, 1, &error, VERDADERO);
						band_cambio_status_2009 = VERDADERO;
           	edo_topes_ult_mov	=	((status_mov_ctrl_volt == E_TOPE_LOGICO_INF) ||
           											(status_mov_ctrl_volt == E_TOPE_LOGICO_SUP) ||
           											(status_mov_ctrl_volt == E_TOPE_FISICO_INF) ||
           											(status_mov_ctrl_volt == E_TOPE_FISICO_SUP) ||
           											(status_mov_ctrl_volt == (E_TOPE_ALCANZADO |E_DURANTE_MOV)));
           	if (((status_mov_ctrl_volt & E_ORDEN_MAN_NULA )!= E_ORDEN_MAN_NULA) &&
              (status_mov_ctrl_volt != E_MOV_INTERR) &&  !edo_topes_ult_mov  ) {
								cambiar_edo_digital(INDICE_STATUS_PTE_BLOQUEADO, 1, &error, VERDADERO); //Puente bloqueado
								band_cambio_status_2000 = VERDADERO;
              }
          }
//					 Si no hubo error y el movimineto fue mayor a 0.01
          if ((status_mov_ctrl_volt == NORMAL)&& (fabsf(prom_mov_reg)> 0.01)) {
          	if (edo_digital(INDICE_DIF_MAX_ORDEN_MENOS_REG, &error, VERDADERO)) {
							cambiar_edo_digital(INDICE_DIF_MAX_ORDEN_MENOS_REG, 0, &error, VERDADERO); // 2009 bit 5
							band_cambio_status_2009 =  VERDADERO;
          	}
						obtener_sem(SEM_GRUPO42, &ptr_sem, &error);
						if (error	==	E_OK) {
							accesar_BD(LEER | ENTERO, 	/*Lee BD 4200 */
		     	  						 GRUPO_ANODOS,
				    						 SG_PARAM_AN_ENT2,
				    						 0,
				    						 0,
			  	  						 sizeof(*ptr_tiempo_estab_luego_mov),
			    							 (void*)ptr_tiempo_estab_luego_mov,
			     							 (void*)&ptr_SG_PARAM_AN_ENT_2);
					  	liberar_sem(SEM_GRUPO42, &ptr_sem, &error);
						}
						obtener_sem(SEM_GRUPO10, &ptr_sem, &error);
						if (error	==	E_OK) {
							accesar_BD(LEER | ENTERO, 	/*Lee BD 10 */
		     	  						 GRUPO_MEDICIONES,
				    						 SG_PARAM_MED_ENT,
				    						 0,
				    						 (SG_PARAM_MED_No_ELEM_ENT-1),
			  	  						 sizeof(grupo_mediciones),
			    							 (void*)grupo_mediciones,
			     							 (void*)&ptr_SG_PARAM_MED_ENT);
			     		liberar_sem(SEM_GRUPO10, &ptr_sem, &error);
						}
//						Espera por 20 seg
            sleep(*ptr_tiempo_estab_luego_mov);  //15 muestreos c/20seg = 5min
            nro_muestras_resist = (*ptr_intervalo_activ_ctrl_volt * 60)/(2* *ptr_tiempo_calc_prom_resist_mov);
            if (nro_muestras_resist == 0) {
              nro_muestras_resist = 1;
            }
            band_calc_cambio_r_mm = FALSO;
            resist_acum          = 0.0;
            for(i=1;i<(nro_muestras_resist + 1);i++) {
            	falla_corriente = edo_digital(INDICE_CAIDA_CORRIENTE, &error, VERDADERO);
            	if (!falla_corriente) {
	  						obtener_sem(SEM_GRUPO13, &ptr_sem, &error);
								if (error	==	E_OK) {
									accesar_BD(LEER | REAL, 	/*Lee BD 1302 Resistencia */
		     	  							 GRUPO_MEDICIONES,
				    							 SG_REG_MED_REAL,
				    							 2,
				    							 2,
			  	  							 sizeof(resist_medida),
			    								 (void*)&resist_medida,
			     								 (void*)&ptr_SG_REG_MED_REAL);
					  			liberar_sem(SEM_GRUPO13, &ptr_sem, &error);
								}
                resist_acum = resist_acum + resist_medida;
								sleep(*ptr_tiempo_calc_prom_resist_mov);
                band_calc_cambio_r_mm = VERDADERO;
              }
              else{
                i               = nro_muestras_resist + 1;
                band_calc_cambio_r_mm = FALSO;
              }
            } /*fin del for*/
						// Lee prioridad de ult Secuencia de mov de anodos
						// para conocer si el ult mov fue de ctrl de voltaje
						obtener_sem(SEM_GRUPO40, &ptr_sem, &error);
						if (error	==	E_OK) {
							accesar_BD(LEER | ENTERO,
	     	  							 GRUPO_ANODOS,
			    							 SG_PARAM_AN_ENT,
			    							 24,
			    							 24,
		  	  							 sizeof(prioridad_ult_mov),
		    								 (void*)&prioridad_ult_mov,
		     								 (void*)&ptr_SG_PARAM_AN_ENT);
			  			liberar_sem(SEM_GRUPO40, &ptr_sem, &error);
						}
						band_prio_ctrl_volt = ((prioridad_ult_mov == PRIOR_MOV_CTRL_VOLT_NORMAL)?1:0);
						// verifica ultima sec de mov por control de voltaje
            if (band_prio_ctrl_volt & band_calc_cambio_r_mm) {
              resist_medida = resist_acum/((float)nro_muestras_resist);
              // calcula cambio de resistencia por milimetro de mov
              resist_por_mm_reg = (resist_medida - resistencia)/prom_mov_reg;
              obtener_sem(SEM_GRUPO21, &ptr_sem, &error);
  						if (error	==	E_OK) {
								accesar_BD(LEER | ENTERO, 	/*Lee BD2117 tiempo desde ultima orden de mov ctrl voltaje*/
	  			  					 	   GRUPO_DIGITAL,
			 		 								 SG_REG_CONT_DIG_ENT,
			   							 		 17,
			   							 		 17,
			   							 		 sizeof(*ptr_tiempo_desde_ult_ord_ctrl),
			   							 		 (void*)ptr_tiempo_desde_ult_ord_ctrl,
			   							     (void*)&ptr_SG_REG_CONT_DIG_ENT);
				  			liberar_sem(SEM_GRUPO21, &ptr_sem, &error);
  						}
              if (*ptr_tiempo_desde_ult_ord_ctrl >= 32768)
                cte_t_filt_resist_mm = 32767;
              else{
              	if (*ptr_tiempo_desde_ult_ord_ctrl < 0)
              		cte_t_filt_resist_mm = 0;
              	else
                	cte_t_filt_resist_mm = (float)(*ptr_tiempo_desde_ult_ord_ctrl);
              }
              // Hace el filtrado de cambio R x mm desplazado
              *ptr_cambio_resist_mm_calc = filtro_analogico(&filtro_mfa,
																					                  resist_por_mm_reg,
                      										                  cte_t_filt_resist_mm,
                      										                  *ptr_cte_t_filtro_resis_mm);
              if (*ptr_cambio_resist_mm_calc <= *ptr_lim_min_cambio_r_mm)
                nro_falla_cambio_r_mm = nro_falla_cambio_r_mm + 1;
              else
                nro_falla_cambio_r_mm = 0;
	            obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
						  if (error	==	E_OK) {
								accesar_BD(ESCRIBIR |REAL, 	/*Escribe BD 4311*/
		     	  						GRUPO_ANODOS,
				    						SG_REG_AN_REAL	,
				    						11,
				    						11,
			  	  						sizeof(*ptr_cambio_resist_mm_calc),
			    							(void*)ptr_cambio_resist_mm_calc,
			      						(void*)&ptr_SG_REG_AN_REAL);
				  			liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
							}
							// Envia cambioR_x mm desplazado_filtrada al sistema supervisor
							formatear_mensaje(MSG_IREA,
  															MENS_ENTERO_REAL,
    														GRUPO_ANODOS,
    														SG_REG_AN_REAL,
    														11,
    														3,
    														1,
    														NULL,
    														ptr_cambio_resist_mm_calc,
    														&mens_salida_ss);
 							result = mq_send(cola_cms100,
    	     										(char *)&mens_salida_ss,
     							 						 sizeof(mens_salida_ss),
     							 						 NULL);
              // Calculo de factor de amortiguamiento
            	amortig = (resist_por_mm_reg - *ptr_cambio_resist_mm_teor);
            	amortig = (amortig / *ptr_cambio_resist_mm_teor) * (*ptr_ganancia_amortig);
            	if (amortig > *ptr_max_tamano_amortig)
              	amortig = *ptr_max_tamano_amortig;
           	 	if ((amortig >  mag_amortig )&& (amortig >= *ptr_min_tamano_amortig)) {
        				mag_amortig_ini =
              	mag_amortig      = (float)(int)amortig;
              	*ptr_tiempo_amortig  = mag_amortig * periodo_muestreo;
              	obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
						  	if (error	==	E_OK) {
									accesar_BD(ESCRIBIR |REAL, 	/*Escribe BD 4319*/
		     	  								GRUPO_ANODOS,
				    								SG_REG_AN_REAL	,
				    								19,
				    								19,
			  	  								sizeof(*ptr_tiempo_amortig),
			    									(void*)ptr_tiempo_amortig,
			      								(void*)&ptr_SG_REG_AN_REAL);
				  				liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
								}
                formatear_mensaje(MSG_IREA,
  																MENS_ENTERO_REAL,
    															GRUPO_ANODOS,
    															SG_REG_AN_REAL,
    															19,
    															3,
    															1,
    															NULL,
    															ptr_tiempo_amortig,
    															&mens_salida_ss);
 								result = mq_send(cola_cms100,
    	     											(char *)&mens_salida_ss,
     							 						 	 sizeof(mens_salida_ss),
     							 						 	 NULL);
            	}
            } /* ultima sec ctrl de voltaje?*/
          }    /*Fin si no hubo error y mov>0.01*/

//        Enviar posicion de puente
					leer_posicion_puente_mm(posicion_puente);
					formatear_mensaje(MSG_POSICION_PTE,
  													MENS_REAL,
    												NULO,
    												NULO,
    												NULO,
    												0,
    												3,
    												NULL,
    												posicion_puente,
    												&mens_salida_ss);
 					result = mq_send(cola_cms100,
    	    								(char *)&mens_salida_ss,
     				 						 	 sizeof(mens_salida_ss),
     				 						 	 NULL);

        break; /*RETORNO*/
        }
				case 2:{  /*Msg proveniente de promedios analogico*/
					obtener_sem(SEM_GRUPO00, &ptr_sem, &error);
  				if (error	==	E_OK) {
						accesar_BD(LEER | ENTERO, 	/*Lee grupo 0*/
	  			  			 	  GRUPO_SISTEMA,
			 		 						SG_PARAM_SIST_ENT,
			   							0,
			   							0,
			   							sizeof(status_alg_ctrl),
			   							(void*)&status_alg_ctrl,
			   							(void*)&ptr_SG_PARAM_SIST_ENT);
		  			liberar_sem(SEM_GRUPO00, &ptr_sem, &error);
  				}
          if ((status_alg_ctrl & BIT_ESTRATEGIA_ANODOS_HABILITADA) !=0)
          	reg_anodos_habilitado = VERDADERO;
          else
          	reg_anodos_habilitado = FALSO;
  				obtener_sem(SEM_GRUPO42, &ptr_sem, &error);
  				if (error	==	E_OK) {
  					accesar_BD(LEER | ENTERO, 	/*Lee grupo 42*/
		     	 				     GRUPO_ANODOS,
				     				   SG_PARAM_AN_ENT2,
				      				 0,
				 				       (SG_PARAM_AN_No_ELEM_ENT2-1),
			  	  				   sizeof(param_ctrl_volt_ent),
			    	    			 (void*)param_ctrl_volt_ent,
			      	 				 (void*)&ptr_SG_PARAM_AN_ENT_2);
   	  			liberar_sem(SEM_GRUPO42, &ptr_sem, &error);
  				}
					obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
  				if (error	==	E_OK) {
  					accesar_BD(LEER | REAL, 	/*Lee grupo 43*/
		     	    			  GRUPO_ANODOS,
				        			SG_REG_AN_REAL,
				        			0,
					      			(SG_REG_AN_REAL_No_ELEM-1),
			    	    			sizeof(param_ctrl_volt_real),
			      	  			(void*)param_ctrl_volt_real,
			        				(void*)&ptr_SG_REG_AN_REAL);
		  			liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
  				}
  				// obtiene status digitales
  				obtener_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
    			status_ctrl_pte							= edo_digital(INDICE_STATUS_CTRL_PT, &error, FALSO);
          falla_motor    							= edo_digital(INDICE_STATUS_FALLA_MOTOR, &error, FALSO);
          puente_bloqueado						= edo_digital(INDICE_STATUS_PTE_BLOQUEADO, &error, FALSO);
          efecto_anodico 							= edo_digital(INDICE_EFECTO_ANODICO, &error, FALSO);
          operac_traseg   						= edo_digital(INDICE_OPERACION_TRASEGADO, &error, FALSO);
          falla_corriente  						= edo_digital(INDICE_CAIDA_CORRIENTE, &error, FALSO);
          status_recup_mov    				= edo_digital(INDICE_SEMAFORO_RECUPERAR_MOV, &error, FALSO);
          band_inic_cuota_mov_sem     = edo_digital(INDICE_SEMAFORO_CUOTA_MOV_ANODOS, &error, FALSO);
					status_equilibrando_puente  = edo_digital(INDICE_EQUILIBRANDO_PUENTE, &error, FALSO);
					status_subida_puente			  = edo_digital(INDICE_SUBIDA_PUENTE, &error, FALSO);
					falla_corr_momentanea       = edo_digital(IND_CAIDA_CORR_MOMENT, &error, FALSO);
					if (error_sem == E_OK) liberar_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);

					obtener_sem(SEM_GRUPO21, &ptr_sem, &error);
  				if (error	==	E_OK) {
						accesar_BD(LEER | ENTERO, 	/*Lee grupo 21*/
	  			  			 	   GRUPO_DIGITAL,
			 		 						 SG_REG_CONT_DIG_ENT,
			   							 0,
			   							 (SG_REG_CONT_DIG_No_ELEM_ENT-1),
			   							 sizeof(digital_teller_word_var),
			   							 (void*)digital_teller_word_var,
			   							 (void*)&ptr_SG_REG_CONT_DIG_ENT);
		  			liberar_sem(SEM_GRUPO21, &ptr_sem, &error);
  				}
					if (*ptr_tiempo_anodo_en_manual >60) { //mas de una hora en manual
						cambiar_edo_digital(INDICE_CTRL_ANODOS_MAS_1HR_MANUAL, 1, &error, VERDADERO);
						codigo_error = E_CTRL_PTE_MAN_1H;
						formatear_mensaje(MSG_ERROR,
  														MENS_ENTERO,
    													ID_CTRL_VOLT,
    													codigo_error,
    													0,
    													2,
    													0,
    													NULL,
    													NULL,
    													&mens_salida_ss);
    				result = mq_send(cola_cms100,
    	    									(char *)&mens_salida_ss,
     				 						 	 	 sizeof(mens_salida_ss),
     				 						 	 	 NULL);
					}
          if (band_inic_cuota_mov || band_inic_cuota_mov_sem) { /*reset c/12Hrs o comando 59, resetear acumuladores*/
            band_inic_cuota_mov = FALSO;
						// ---------  envio msg               E_ORDEN_MAN_NULA
						elemento_ini = 21;
						elemento_fin = 27;
						for(i=elemento_ini; i<(elemento_fin+1);i++) {
							formatear_mensaje(MSG_IREA,
  															MENS_ENTERO_REAL,
    														GRUPO_ANODOS,
    														SG_REG_AN_REAL,
    														i,
    														3,
    														1,
    														NULL,
    														&param_ctrl_volt_real[i],
    														&mens_salida_ss);
     					result = mq_send(cola_cms100,
       		   						(char *)&mens_salida_ss,
     										sizeof(mens_salida_ss),
     										NULL );
						}
    				*ptr_mov_anodo_acum_auto       = 0.0;
            *ptr_mov_anodo_acum_auto_abajo = 0.0;
            *ptr_mov_anod_acum_reg         = 0.0;
            *ptr_mov_anod_acum_reg_abajo   = 0.0;
            *ptr_mov_anod_acum_equil       = 0.0;
            band_ext_cuota         			   = FALSO;
            band_cambio_status_2009        = VERDADERO;
            obtener_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
            cambiar_edo_digital(INDICE_MOV_MAX_PERMITIDO_DIA, 0, &error, FALSO);
            cambiar_edo_digital(INDICE_MOV_MAX_DESEQ_DIA, 0, &error, FALSO);
            cambiar_edo_digital(INDICE_SEMAFORO_CUOTA_MOV_ANODOS, 0, &error, FALSO);
            if (error_sem == E_OK) liberar_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
          }
          if (status_recup_mov) {
          	nro_falla_cambio_r_mm = 0;
            filtro_mfa.modo = 0;
					  *ptr_cambio_resist_mm_calc = filtro_analogico(&filtro_mfa,
																				                  *ptr_cambio_resist_mm_teor,
                      									                  1.0,
                      									                  1.0);
            formatear_mensaje(MSG_IREA,
  														MENS_ENTERO_REAL,
    													GRUPO_ANODOS,
    													SG_REG_AN_REAL,
    													11,
    													3,
    													1,
    													NULL,
    													ptr_cambio_resist_mm_calc,
    													&mens_salida_ss);
 						result = mq_send(cola_cms100,
    	     										(char *)&mens_salida_ss,
     							 						 sizeof(mens_salida_ss),
     							 						 NULL);
            cambiar_edo_digital(INDICE_SEMAFORO_RECUPERAR_MOV, 0, &error, VERDADERO);
          }
          calculo_resistencia_ref(&banda_sup_x_radic,
													 				ptr_resist_ref_calculada,
													 		 		ptr_band_sum_r_arranque, //4206
													 				&band_ext_cuota_mov,
													 				*ptr_tiempo_desde_arranque,
													 				*ptr_resist_ref, //4302
													 				*ptr_banda_adic_con_resit_adic);//4337

					if ((status_alg_ctrl & BIT_CTRL_ADAPTATIVO_HABILITADO )&&
             (status_alg_ctrl & BIT_ESTRATEGIA_ANODOS_HABILITADA)) {
            obtener_sem(SEM_GRUPO63, &ptr_sem, &error);
            if (error	==	E_OK) {
							accesar_BD(LEER | REAL, 	/*Leer BD6305*/
		     	  						GRUPO_CTRLADAPTATIVO	,
				    						SG_REG_AD_REAL,
				    						5,
				    						5,
			  	  						sizeof(b1_promedio),
			    							(void*)&b1_promedio,
			      						(void*)&ptr_SG_REG_AD_REAL);
				  		liberar_sem(SEM_GRUPO63, &ptr_sem, &error);
					  }
					  banda_sup_calc_b1 = salida_lineal(*ptr_lim_banda_adapt_1,
                                                *ptr_lim_banda_adapt_b1_X0,
                                                *ptr_lim_banda_adapt_2,
                                                *ptr_lim_banda_adapt_b1,
                                                b1_promedio);
            obtener_sem(SEM_GRUPO62, &ptr_sem, &error);
            if (error	==	E_OK) {
							accesar_BD(LEER | ENTERO, 	/*Leer BD6203*/
		     	  						GRUPO_CTRLADAPTATIVO	,
				    						SG_REG_AD_ENT,
				    						3,
				    						3,
			  	  						sizeof(contador_tracking),
			    							(void*)&contador_tracking,
			      						(void*)&ptr_SG_REG_AD_ENT);
				  		liberar_sem(SEM_GRUPO62, &ptr_sem, &error);
					  }
            if ((contador_tracking != 0) && (contador_tracking > -6))
            	*ptr_banda_sup_ctrl = banda_sup_x_radic + *ptr_resist_adic_track;
            else
            	*ptr_banda_sup_ctrl = banda_sup_x_radic + banda_sup_calc_b1;
            if (*ptr_banda_sup_ctrl < 0.0)
              *ptr_banda_sup_ctrl = 0.0;
            *ptr_banda_sup_ctrl  = *ptr_resist_ref_calculada + *ptr_banda_sup_ctrl;
            *ptr_banda_inf_ctrl = *ptr_resist_ref_calculada
                                  - banda_sup_calc_b1/ *ptr_factor_adap_banda_inf;
          }
          else{
            *ptr_banda_sup_ctrl  = *ptr_resist_ref_calculada
                                          + *ptr_banda_sup_fija
                                          + banda_sup_x_radic;
            *ptr_banda_inf_ctrl = *ptr_resist_ref_calculada
                                  - *ptr_banda_inf_fija;
          }

          if (mag_amortig > 2.0) {
          	mag_amortig   	 = mag_amortig - 1.0;
            *ptr_tiempo_amortig = mag_amortig * periodo_muestreo;
            factor_amortig 	 = salida_lineal(*ptr_factor_min_amortig,  // y(1)
                           	            		  mag_amortig_ini,         // x(1)
                            	           		  *ptr_factor_max_amortig, // y(0)
                              	         		  1.0,   							     // x(0)
                                	       		  mag_amortig);            // x
          }
          else{
            *ptr_tiempo_amortig = 0.0;
            factor_amortig   = 1.0;
          }
          if (falla_corriente) {
           	cambiar_edo_digital(INDICE_FALLA_CORRIENTE_ANODOS, 1, &error, VERDADERO);
           	band_cambio_status_2009 = VERDADERO;
          }
          else{
           	if (edo_digital(INDICE_FALLA_CORRIENTE_ANODOS, &error, VERDADERO)) {
           		cambiar_edo_digital(INDICE_FALLA_CORRIENTE_ANODOS, 0, &error, VERDADERO);
           		band_cambio_status_2009 = VERDADERO;
           	}
          }
			    leer_posicion_puente_mm(posicion_puente);
          /* Test de desnivel de puente*/
         	if (No_MOTORES == 2) {
          	dif_entre_motores = (posicion_puente[0] - posicion_puente[1]);
          	if (fabsf(dif_entre_motores) >= *ptr_max_inclinacion_falla) {
          		obtener_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
          		cambiar_edo_digital(INDICE_STATUS_PTE_BLOQUEADO, 1, &error, FALSO);
            	cambiar_edo_digital(INDICE_PUENTE_INCLINADO, 1, &error, FALSO);
            	band_cambio_status_2009 = band_cambio_status_2000 = VERDADERO;
            	if (error_sem == E_OK) liberar_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
            	reg_anodos_habilitado = FALSO;
          //**************prender LEDS DE PUENTE INCLINADO Y BLOQUEADO*******
          // enviar FM desnivel logico
          		codigo_error = E_PUENTE_INCLINADO_LOGICO;
            	formatear_mensaje(MSG_ERROR,
	 								 					  MENS_ENTERO,
					      							ID_CTRL_VOLT,
    													codigo_error,
    							 						0,
    							 						2,
    							 						0,
					      						  NULL,
					      						  NULL,
    													&mens_salida_ss);
					  	result = mq_send(cola_cms100,
    	     										(char *)&mens_salida_ss,
     							 						 sizeof(mens_salida_ss),
     							 						 NULL);
          	}
          	else{
          		if (edo_digital(INDICE_PUENTE_INCLINADO, &error, VERDADERO)) {
          			cambiar_edo_digital(INDICE_PUENTE_INCLINADO, 0, &error, VERDADERO);
          			band_cambio_status_2009 = VERDADERO;
          		}
          	}
					}//  fin de test de desnivel
					//valida caso cuando  nunca ha tenido caida de corriente pues valor es -1
					tiempo_desde_caida_corr=  (*ptr_tiempo_desde_caida_corr > 0)? *ptr_tiempo_desde_caida_corr:30;
					tiempo_desde_bajon_corr=  (*ptr_tiempo_desde_bajon_corr > 0)? *ptr_tiempo_desde_bajon_corr:30;
					if ((tiempo_desde_bajon_corr > 0) && (tiempo_desde_bajon_corr < 20)) {
					  codigo_error = E_PAUSA_BAJON_CORRIENTE;
            formatear_mensaje(MSG_ERROR,
	 								 					  MENS_ENTERO,
					      							ID_CTRL_VOLT,
    													codigo_error,
    							 						0,
    							 						2,
    							 						0,
					      						  NULL,
					      						  NULL,
    													&mens_salida_ss);
					  result = mq_send(cola_cms100,
    	     										(char *)&mens_salida_ss,
     							 						 sizeof(mens_salida_ss),
     							 						 NULL);
					}
        	if (reg_anodos_habilitado && status_ctrl_pte && !efecto_anodico && !operac_traseg &&
        	   !falla_motor && !puente_bloqueado && !falla_corriente && !falla_corr_momentanea &&
        	   !status_equilibrando_puente && !status_subida_puente && tiempo_desde_caida_corr>20
        	   && tiempo_desde_bajon_corr>20 ) {
          	if (resist_medida <= *ptr_min_resist) {
          		band_cambio_status_2009 = VERDADERO;
          		cambiar_edo_digital(INDICE_RESISTENCIA_MENOR_MINIMO, 1, &error, VERDADERO);
           		// verifica nivelar puente
							codigo_error = E_RESIST_MEN_MIN;
		          formatear_mensaje(MSG_ERROR,
	 								 					  		MENS_ENTERO,
					      									ID_CTRL_VOLT,
    															codigo_error,
    							 								0,
    							 								2,
    							 								0,
					      						  		NULL,
					      						  		NULL,
    															&mens_salida_ss);
					  	result = mq_send(cola_cms100,
    	     													(char *)&mens_salida_ss,
     							 									sizeof(mens_salida_ss),
     							 									NULL);
							if (No_MOTORES == 2) {
								bandera_escribir_BD = FALSO;
       					Nivelar_Puente(ptr_orden_mov_anodo,
																 dif_entre_motores,
								  							 &bandera_escribir_BD,
																 *ptr_lim_inclinacion_pte ,
																 *ptr_mov_anod_acum_equil,
																 *ptr_max_cuota_mov_equil_12hrs);
								prioridad_orden_mov = PRIOR_MOV_CTRL_VOLT_EQUIL;
							}
							else
								bandera_escribir_BD = VERDADERO;
          	}// fin si resistencia menor a min
          	else{
          		bandera_escribir_BD = FALSO;
							if (edo_digital(INDICE_RESISTENCIA_MENOR_MINIMO, &error, VERDADERO)) {
          			obtener_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
          			cambiar_edo_digital(INDICE_RESISTENCIA_MENOR_MINIMO, 0, &error, FALSO);
          			cambiar_edo_digital(INDICE_SEMAFORO_RESIST_FILT , 1, &error, FALSO);
          			band_cambio_status_2009 = VERDADERO;
          			if (error_sem == E_OK) liberar_sem(SEM_GRUPO20, &ptr_sem20, &error_sem);
          			//Enviar TK11
							}
	          	if (band_ext_cuota_mov)
	          		band_ext_cuota = VERDADERO;
	          	if (band_ext_cuota)
	          		max_cuota_mov_reg = *ptr_incr_cuota_arranque * *ptr_max_cuota_mov_12hrs;
	          	else
	           		max_cuota_mov_reg = *ptr_max_cuota_mov_12hrs;
	          	if (*ptr_mov_anodo_acum_auto <= max_cuota_mov_reg) {
				    		if (*ptr_tiempo_desde_ult_ord_ctrl > *ptr_max_t_falla_resist_mm)
				      		nro_falla_cambio_r_mm = 0;
				      	if (*ptr_tiempo_desde_ult_ord_ctrl < *ptr_t_max_error_resist_mm) {
				      		if (nro_falla_cambio_r_mm >= *ptr_nro_max_falla_resist_mm) {
				        		bandera_escribir_BD = VERDADERO;
				        		band_cambio_status_2009 = VERDADERO;
	          				cambiar_edo_digital(INDICE_RPS_BAJA, 1, &error, VERDADERO);
	          				//							        ENVIAR MSG FM 2123
	          				formatear_mensaje(MSG_ERROR,
 								 					  				  MENS_ENTERO,
				      											  ID_CTRL_VOLT,
  																	  E_RPS_BAJO,
  							 										  0,
  							 										  2,
  							 										  0,
				      						  				  NULL,
				      						  				  NULL,
  																	  &mens_salida_ss);
				  					result = mq_send(cola_cms100,
  	     													(char *)&mens_salida_ss,
   							 									sizeof(mens_salida_ss),
   							 									NULL);

				        	}
				        	else{
				        		if (edo_digital(INDICE_RPS_BAJA, &error, VERDADERO)) {
				      				cambiar_edo_digital(INDICE_RPS_BAJA, 0, &error, VERDADERO);
				      				band_cambio_status_2009 = VERDADERO;
				        		}
				        	}
				      	}
				      	else{
				      		if (edo_digital(INDICE_RPS_BAJA, &error, VERDADERO)) {
				      			cambiar_edo_digital(INDICE_RPS_BAJA, 0, &error, VERDADERO);
				      			band_cambio_status_2009 = VERDADERO;
				        	}
				      	}
	          	}
	          	else{
	          		bandera_escribir_BD = VERDADERO;
	          		cambiar_edo_digital(INDICE_MOV_MAX_PERMITIDO_DIA, 1, &error, VERDADERO);
	          		band_cambio_status_2009 = VERDADERO;
	          		//envia MSG Error
		          	formatear_mensaje(MSG_ERROR, MENS_ENTERO, ID_CTRL_VOLT, E_MAX_MOV_12H,
 										            0, 2, 0, NULL, NULL, &mens_salida_ss);
								result = mq_send(cola_cms100,
															(char *)&mens_salida_ss,
 															sizeof(mens_salida_ss),
 															NULL);

	          	}
		          if (!bandera_escribir_BD) {		/*Actualizar status de Ctrl Voltaje*/
		          //Resistenciafiltrada 20 min
  			      	if (resist_filtrada.corta > *ptr_banda_sup_ctrl ) {
									if (!edo_digital(INDICE_RF20_MAYOR_BANDA_SUP, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_RF20_MAYOR_BANDA_SUP, 1, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
									}
					      }
					      else{
					      	if (edo_digital(INDICE_RF20_MAYOR_BANDA_SUP, &error, VERDADERO)) {
					       		cambiar_edo_digital(INDICE_RF20_MAYOR_BANDA_SUP, 0, &error, VERDADERO);
					       		band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      if (resist_filtrada.corta < *ptr_banda_inf_ctrl) {
									if (!edo_digital(INDICE_RF20_MENOR_BANDA_INF, &error, VERDADERO)) {
					       		cambiar_edo_digital(INDICE_RF20_MENOR_BANDA_INF, 1, &error, VERDADERO);
					       		band_cambio_status_2009 = VERDADERO;
									}
					      }
					      else{
					      	if (edo_digital(INDICE_RF20_MENOR_BANDA_INF, &error, VERDADERO)) {
						      	cambiar_edo_digital(INDICE_RF20_MENOR_BANDA_INF, 0, &error, VERDADERO);
						      	band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      //Resistenciafiltrada 90 min
					      if (resist_filtrada.larga > *ptr_banda_sup_ctrl ) {
									if (!edo_digital(INDICE_RF90_MAYOR_BANDA_SUP, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_RF90_MAYOR_BANDA_SUP, 1, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
									}
					      }
					      else{
					      	if (edo_digital(INDICE_RF90_MAYOR_BANDA_SUP, &error, VERDADERO)) {
					       		cambiar_edo_digital(INDICE_RF90_MAYOR_BANDA_SUP, 0, &error, VERDADERO);
					       		band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      if (resist_filtrada.larga < *ptr_banda_inf_ctrl) {
									if (!edo_digital(INDICE_RF90_MENOR_BANDA_INF, &error, VERDADERO)) {
					       		cambiar_edo_digital(INDICE_RF90_MENOR_BANDA_INF, 1, &error, VERDADERO);
					       		band_cambio_status_2009 = VERDADERO;
									}
					      }
					      else{
					      	if (edo_digital(INDICE_RF90_MENOR_BANDA_INF, &error, VERDADERO)) {
					       		cambiar_edo_digital(INDICE_RF90_MENOR_BANDA_INF, 0, &error, VERDADERO);
					       		band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      //Bandera de chequeo de tiempos y resistencia
					      //Se pregunta por t desde arranque si status arranque en 1
					      //si no tiempo desde arranque se asigna valor
								if (!edo_digital(INDICE_STATUS_ARRANQUE, &error, VERDADERO)) {
									*ptr_tiempo_desde_arranque = *ptr_t_min_espera_luego_arranq + 1;
								}
					       band_dif_resist =((fabsf(resist_medida - resist_filtrada.larga) > *ptr_maxima_variacion_resis)&&
					                        (*ptr_tiempo_desde_arranque > *ptr_t_min_espera_luego_arranq)&&
					                        (*ptr_tiempo_desde_traseg  > *ptr_t_min_espera_luego_tras ));

				        if (band_dif_resist) {
									if (!edo_digital(INDICE_DIFR_TIEMPOS_TRAS_ARR, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_DIFR_TIEMPOS_TRAS_ARR, 1, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
									}
				        }
					      else{
					      	if (edo_digital(INDICE_DIFR_TIEMPOS_TRAS_ARR, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_DIFR_TIEMPOS_TRAS_ARR, 0, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      band_tiempo_ea_min =	((*ptr_tiempo_desde_ea < *ptr_t_min_espera_luego_ea) &&
					      											(*ptr_tiempo_desde_ea > 0));
					      if (band_tiempo_ea_min) {
									if (!edo_digital(INDICE_TIEMPO_DESDE_EA_MENOR_MIN, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_TIEMPO_DESDE_EA_MENOR_MIN, 1, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
									}
					      }
					      else{
					      	if (edo_digital(INDICE_TIEMPO_DESDE_EA_MENOR_MIN, &error, VERDADERO)) {
					      		cambiar_edo_digital(INDICE_TIEMPO_DESDE_EA_MENOR_MIN, 0, &error, VERDADERO);
					      		band_cambio_status_2009 = VERDADERO;
					      	}
					      }
					      //Chequea condiciones para ordenar mov hacia arriba  o hacia abajo
				        if (((resist_filtrada.corta < *ptr_banda_inf_ctrl)&&
				        	(resist_medida < *ptr_banda_inf_ctrl))||
				        	((resist_filtrada.larga >= *ptr_banda_sup_ctrl)&&
				          (resist_filtrada.corta >= *ptr_banda_sup_ctrl)&&
				          (resist_medida >= *ptr_banda_sup_ctrl)&&
				          (!band_tiempo_ea_min)&& (!band_dif_resist))) {
									regulador(ptr_orden_mov_anodo,
														*ptr_cambio_resist_mm_teor,
														*ptr_ganancia_regular_mov,
														*ptr_resist_ref_calculada,
														resist_medida,
														factor_amortig,
														*ptr_min_mov_ordenado,
														*ptr_max_mov_ordenado,
														&bandera_escribir_BD);
									prioridad_orden_mov = PRIOR_MOV_CTRL_VOLT_NORMAL;
					      }
					      else{
					       	if (No_MOTORES == 2) {
					       		Nivelar_Puente(ptr_orden_mov_anodo,
																	 dif_entre_motores,
							  						 			 &bandera_escribir_BD,
													 				 *ptr_lim_inclinacion_pte ,
													 				 *ptr_mov_anod_acum_equil,
													 				 *ptr_max_cuota_mov_equil_12hrs);;
										prioridad_orden_mov = PRIOR_MOV_CTRL_VOLT_EQUIL;
					       	}
					       	else{
					       		bandera_escribir_BD = VERDADERO;
					       	}
					      }// fin de Nivelar
		          }
	          }
          	if (!bandera_escribir_BD) { /*Enviar orden de regulacion o nivelacion*/
//							Enviar posicion de puente
							leer_posicion_puente_mm(posicion_puente);
							formatear_mensaje(MSG_POSICION_PTE,
  															MENS_REAL,
    														NULO,
    														NULO,
    														NULO,
    														0,
    														3,
    														NULL,
    														posicion_puente,
    														&mens_salida_ss);
 							result = mq_send(cola_cms100,
    	    								(char *)&mens_salida_ss,
     				 						 	 sizeof(mens_salida_ss),
     				 						 	 NULL);
     				 	msg_ctrl_sec_mov.mens_cv.tipo	 = MENS_NUEVO;
							msg_ctrl_sec_mov.mens_cv.orden = ARRANCAR;
							msg_ctrl_sec_mov.mens_cv.prio  = prioridad_orden_mov;
							msg_ctrl_sec_mov.mens_cv.sec_elegida = SEC_MOV_NORMAL;
							for(i=0;i<No_MOTORES;i++)
								msg_ctrl_sec_mov.mens_cv.mov_ord_mm[i] = ptr_orden_mov_anodo[i];
							if (ptr_orden_mov_anodo[0] != 0) {  // valida que la orden sea distinta de cero
								status_envio= mq_send(cola_ctrl_secuencia_mov,
						          	 	 			  	 (char *) &msg_ctrl_sec_mov,
         									    		 	 sizeof(msg_ctrl_sec_mov),
         	  	  		 								 0);
		       	  	if (status_envio != E_FALLA ) {
		         	  	if (prioridad_orden_mov == PRIOR_MOV_CTRL_VOLT_EQUIL)
		                *ptr_mov_anod_acum_equil = *ptr_mov_anod_acum_equil + fabsf(ptr_orden_mov_anodo[0]) + fabsf(ptr_orden_mov_anodo[1]);
		         	  	if (prioridad_orden_mov == PRIOR_MOV_CTRL_VOLT_NORMAL) {
		              	*ptr_mov_anodo_acum_auto = *ptr_mov_anodo_acum_auto + fabsf(ptr_orden_mov_anodo[0]);
						        if (ptr_orden_mov_anodo[0] < 0.0)
						        	*ptr_mov_anodo_acum_auto_abajo = *ptr_mov_anodo_acum_auto_abajo + fabs(ptr_orden_mov_anodo[0]);
		         	  	}
		         	  	*ptr_tiempo_desde_ult_ord_ctrl = 0;
		         	  	obtener_sem(SEM_GRUPO21, &ptr_sem, &error);
									if (error	==	E_OK) {
										accesar_BD(ESCRIBIR | ENTERO, 	/*COMIENZA conteo ultima orden de mov ctrl voltaje*/
		  			  					 	   GRUPO_DIGITAL,
				 		 								 SG_REG_CONT_DIG_ENT,
				   							 		 17,
				   							 		 17,
				   							 		 sizeof(*ptr_tiempo_desde_ult_ord_ctrl),
				   							 		 (void*)ptr_tiempo_desde_ult_ord_ctrl,
				   							     (void*)&ptr_SG_REG_CONT_DIG_ENT);
					  				liberar_sem(SEM_GRUPO21, &ptr_sem, &error);
									}
		       	  	}// fin de chequeo en envio de msg de salida
          		} //solo envia ordenes distintas de 0
          	}// fin enviar orden de regulacion o nivelacion
          }// fin de envio orden
					/* Escribir BD*/
					obtener_sem(SEM_GRUPO43, &ptr_sem, &error);
  				if (error	==	E_OK) {
    				accesar_BD(ESCRIBIR  | REAL, 	/*Escribe 4305 4306 y 4307*/
		    	 			   	   GRUPO_ANODOS,
						  	 		 	 SG_REG_AN_REAL,
			  	     			 	 5,
						   				 7,
		   		  	 		 	 	(sizeof(*ptr_resist_ref_calculada)*3),
			    				 	 	(void*)&param_ctrl_volt_real[5],
		         	 		   	(void*)&ptr_SG_REG_AN_REAL);
    				accesar_BD(ESCRIBIR  | REAL, 	/*Escribe 4311 */
		    	 			   	   GRUPO_ANODOS,
						  	 		 	 SG_REG_AN_REAL,
			  	     			 	 11,
						   			 	 11,
		   		  	 		 		 sizeof(*ptr_cambio_resist_mm_calc),
			    				 	 	(void*)ptr_cambio_resist_mm_calc,
		         	 		   	(void*)&ptr_SG_REG_AN_REAL);
    				accesar_BD(ESCRIBIR  | REAL, 	/*Escribe 4319 */
		    	 			   	   GRUPO_ANODOS,
						  	 		 	 SG_REG_AN_REAL,
			  	     			 	 19,
						   			 	 19,
		   		  	 		 		 sizeof(*ptr_tiempo_amortig ),
			    				 		 (void*)ptr_tiempo_amortig ,
		         	 		   	 (void*)&ptr_SG_REG_AN_REAL);
						accesar_BD(ESCRIBIR  | REAL, 	/*Escribe del 4321 al 4325 */
		    	 			   	   GRUPO_ANODOS,
						  	 		 	 SG_REG_AN_REAL,
			  	     			 	 21,
						   			 	 25,
		   		  	 		 		 (sizeof(*ptr_mov_anodo_acum_auto)*5),
			    				 	 	 (void*)&param_ctrl_volt_real[21],
		         	 		   	 (void*)&ptr_SG_REG_AN_REAL);
						accesar_BD(ESCRIBIR  | REAL, 	/*Escribe del 4329 y 4330 */
		    	 			   	   GRUPO_ANODOS,
						  	 		 	 SG_REG_AN_REAL,
			  	     			 	 29,
						   			 	 30,
		   		  	 		 		 (sizeof(*ptr_orden_mov_anodo)*2),
			    				 	 	 (void*)&param_ctrl_volt_real[29],
		         	 		   	 (void*)&ptr_SG_REG_AN_REAL);
         		liberar_sem(SEM_GRUPO43, &ptr_sem, &error);
  				}
      	 	resistencia = resist_medida;
        	if (*ptr_band_envio_inf_ss) {
    				formatear_mensaje(MSG_IREA,          //envio de resistencia 10 min BD1330
						 								 MENS_ENTERO_REAL,
						 								 GRUPO_MEDICIONES,
						 								 SG_REG_MED_REAL,
						 								 30,
						 								 3,
														 1,
														 NULL,
														 &resist_medida,
														 &mens_salida_ss);
   					result = mq_send(cola_cms100,
     	   										(char *)&mens_salida_ss,
   													sizeof(mens_salida_ss),
   													NULL );
						elemento_ini = 7;
						elemento_fin = 8;  //envio 1307 y 1308
						info = resist_filtrada.corta;
						for(i=elemento_ini; i<(elemento_fin+1);i++) {
							formatear_mensaje(MSG_IREA,
  															MENS_ENTERO_REAL,
    														GRUPO_MEDICIONES,
    														SG_REG_MED_REAL,
    														i,
    														3,
    														1,
    														NULL,
    														&info,
    														&mens_salida_ss);
       					result = mq_send(cola_cms100,
         	   										(char *)&mens_salida_ss,
       													sizeof(mens_salida_ss),
       													NULL );
       					info = resist_filtrada.larga;
							}

          	}
          	if (fabsf(*ptr_resist_ref - *ptr_resist_ref_calculada) >= 0.1)
            	band_inic_resit_ref = VERDADERO;
          	else{
            	if (fabsf(*ptr_resist_ref - resist_ref_sist) >= 0.1) {
            		resist_ref_sist = *ptr_resist_ref;
              	band_inic_resit_ref = VERDADERO;
            	}
            	else
            		band_inic_resit_ref = FALSO;
          	}
          	if (*ptr_band_envio_inf_ss || band_inic_resit_ref || band_inic_resit_ref_ant) {
							formatear_mensaje(MSG_IREA,
    														MENS_ENTERO_REAL,
      													GRUPO_ANODOS,
      													SG_REG_AN_REAL,
      													5,
      													3,
      													1,
      													NULL,
      													ptr_resist_ref_calculada,
      													&mens_salida_ss);
     					result = mq_send(cola_cms100,
         		  								(char *)&mens_salida_ss,
       												sizeof(mens_salida_ss),
       												NULL );

          	}
          	band_inic_resit_ref_ant = band_inic_resit_ref;

          	if (*ptr_band_envio_inf_ss) {
 //         	Envio msg  4306 y 4307
						elemento_ini = 6;
						elemento_fin = 7;
						for(i=elemento_ini; i<(elemento_fin+1);i++) {
          		formatear_mensaje(MSG_IREA, MENS_ENTERO_REAL, GRUPO_ANODOS, SG_REG_AN_REAL,
    														i, 3, 1, NULL, &param_ctrl_volt_real[i], &mens_salida_ss);
   						result = mq_send(cola_cms100,
       		  									(char *)&mens_salida_ss,
     													sizeof(mens_salida_ss),
     													NULL );
         		}
       		}
				break;
				}/*Fin Nueva Orden;*/
			}/*Fin switch;*/
			if (band_cambio_status_2000) {
				obtener_sem(SEM_GRUPO20, &ptr_sem, &error);
				if (error	==	E_OK) {
					accesar_BD(LEER | ENTERO, 	/*Lee BD 2000*/
	  			 	   			GRUPO_DIGITAL,
	 						 			SG_REG_EDOS_DIG_ENT,
							 			0,
							 			0,
							 			sizeof(status_digital),
							 			(void*)&status_digital,
							 			(void*)&ptr_SG_REG_EDOS_DIG_ENT);
					liberar_sem(SEM_GRUPO20, &ptr_sem, &error);
				}
			  formatear_mensaje(MSG_IENT, MENS_ENTERO, GRUPO_DIGITAL, SG_REG_EDOS_DIG_ENT,
    										  0, 4, 0, &status_digital, NULL, &mens_salida_ss);
				result = mq_send(cola_cms100,
    	   				 				 (char *)&mens_salida_ss,
     									   sizeof(mens_salida_ss),
     							 			 NULL);
     		band_cambio_status_2000 = FALSO;
			}
			if (band_cambio_status_2009) {
				obtener_sem(SEM_GRUPO20, &ptr_sem, &error);
				if (error	==	E_OK) {
					accesar_BD(LEER | ENTERO, 	/*Lee BD 2009*/
	  			 	  			 GRUPO_DIGITAL,
				 						 SG_REG_EDOS_DIG_ENT,
										 9,
										 9,
										 sizeof(status_digital),
										 (void*)&status_digital,
										 (void*)&ptr_SG_REG_EDOS_DIG_ENT);
					liberar_sem(SEM_GRUPO20, &ptr_sem, &error);
				}
	      formatear_mensaje(MSG_IENT, MENS_ENTERO, GRUPO_DIGITAL, SG_REG_EDOS_DIG_ENT,
    										  9, 4, 0, &status_digital, NULL, &mens_salida_ss);
				 result = mq_send(cola_cms100,
    	   				 				 (char *)&mens_salida_ss,
     									   sizeof(mens_salida_ss),
     							 			 NULL);
     		 band_cambio_status_2009 = FALSO;
		 	}
     } /*si status recepcion es OK*/
   }//*Fin del loop por siempre*/


	return EXIT_SUCCESS;
}

