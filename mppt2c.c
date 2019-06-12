/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 * Copyright 1990-2013 The MathWorks, Inc.
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 *您必须将S_FUNCTION_NAME指定为S-function的名称
?*（即将sfuntmpl_basic替换为S函数的名称）。
 */

#define S_FUNCTION_NAME  mppt2c
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 *需要包含simstruc.h来定义SimStruct和
?*其相关的宏定义。
 */
#include "simstruc.h"
#include "math.h"

double P_old = 0;
double U_old = 0;
double I_old = 0;
double slope = 0;
const double n = 0.00004;
double pnew = 0;
double dP = 0;
double dU = 0;
double dI = 0;
double outD = 0;
double m, k1,Unew, Inew;

/* Error handling
 * --------------
/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 *Simulink使用尺寸信息来确定S函数,初始化
  *块的特征（输入，输出，状态等的数量）
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
 
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    /*指定块具有的输入端口数*/
    if (!ssSetNumInputPorts(S, 2)) return;
    //设置input 数据类型
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    //指定块端口的直接馈通状态
      /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    //指定进入端口的信号元素必须是连续的,即存贮地址是否连续的 
    ssSetInputPortRequiredContiguous(S, 0, 1); 
    ssSetInputPortRequiredContiguous(S, 1, 1);
 
    //设置数据维度数 int_T ssSetInputPortWidth(SimStruct *S, int_T port, int_T width)
    ssSetInputPortWidth(S, 0, 1); 
    ssSetInputPortWidth(S, 1, 1);

    /*指定块具有的output端口数*/
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortWidth(S, 0, 1);
    
    ssSetNumSampleTimes(S, 1);// s指定S-Function块具有的采样次数
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 *  此功能用于指定您的采样时间
 *  S功能。您必须注册相同数量的采样时间
 *  在ssSetNumSampleTimes中指定
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.0004);//INHERITED_SAMPLE_TIME指定继承其样本的块*来自提供输入的块的时间。
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
   
    real_T *y = ssGetOutputPortSignal(S,0);
    const real_T *PV_I = (const real_T*) ssGetInputPortSignal(S,0);
    const real_T *PV_U = (const real_T*) ssGetInputPortSignal(S,1);
   
	Unew = *PV_U;
	Inew = *PV_I;
	dU = Unew - U_old;
	dI = Inew - I_old;
	pnew = Unew*Inew;
    //ssPrintf("unew is %g inew is %g  \n",Unew,Inew);
	dP = pnew - P_old;
	if(dU != 0){
		m = dP/dU;
		k1 = n*abs(m);
       
	}
	else k1 = 0.05;
	if(k1 > 0.05) k1 = 0.05;
    ssPrintf("M: is %g K1: %g \n",m, k1);
	if( dU == 0){
		if(dI ==0)slope =0;
		if(dI > 0) slope = -1.0;
		else slope = 1.0;			
	}
	else{
		if(dI/dU == (-Inew/Unew)){
            slope = 0;
            ssPrintf("达到最佳值  \n");
        }
		if(dI/dU > (-Inew/Unew))slope = -1.0;
		else slope = 1.0; 
	}
	// 调整占空比D
    outD = outD + k1*slope;
    //更新状态
	U_old = Unew;
	I_old = Inew;
	P_old = pnew;
    if(outD <= 0.001 )outD = 0.001;
    if(outD >= 0.999)outD = 0.999;
    ssPrintf("dd is %g \n",outD);
    *y = outD;            
}

#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
      
      
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 *?*摘要：
?*在此功能中，您应执行任何必要的操作
?*在模拟结束时。例如，如果内存是
?*在mdlStart中分配，这是释放它的地方。
 */
static void mdlTerminate(SimStruct *S)
{
 P_old = 0;
 U_old = 0;
 I_old = 0;
 outD = 0;
 m =0 ;
 k1 = 0;   
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
