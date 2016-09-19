/*
 * TODO:    Hinderniss einfügen durchleiten.
 *          Wenn Ziel nicht erreichbar stürzt Simulink ab.
 *          Wenn Ziel = Start absturz!!!!
 **/



 /*
  * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
  *
  *  -------------------------------------------------------------------------
  *  | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
  *  -------------------------------------------------------------------------
  *
  * Copyright 1990-2002 The MathWorks, Inc.
  * $Revision: 1.27.4.2 $
  */


  /*
   * You must specify the S_FUNCTION_NAME as the name of your S-function
   * (i.e. replace sfuntmpl_basic with the name of your S-function).
   */

#define S_FUNCTION_NAME  bahnplaner_v2
#define S_FUNCTION_LEVEL 2

   /*
	* Need to include simstruc.h for the definition of the SimStruct and
	* its associated macro definitions.
	*/
#include "simstruc.h"
#include <stdio.h>
#include <malloc.h>
#include <math.h>

	//-------------------------------------------------------------------
	// Defines für Astern Implementierung
	//-------------------------------------------------------------------
#define FALSE 0
#define TRUE 1
#define MAXX 10
#define MAXY 10
#define MAX_LIST_ELEMENTS 20
#define SIZEDELTA1 4
#define SIZEDELTA2 2
#define COST 1
#define DEFAULTWAYPOINT -1
// Konsolenausgabe aktivieren:
//#define PRINTGRID

//-------------------------------------------------------------------
//-------------------------------------------------------------------

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

 /*====================*
  * S-function methods *
  *====================*/

  /* Function: mdlInitializeSizes ===============================================
   * Abstract:
   *    The sizes information is used by Simulink to determine the S-function
   *    block's characteristics (number of inputs, outputs, states, etc.).
   */
static void mdlInitializeSizes(SimStruct *S)
{
	/* See sfuntmpl_doc.c for more details on the macros below */

	ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		/* Return if number of expected != number of actual parameters */
		return;
	}

	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);

	if (!ssSetNumInputPorts(S, 3)) return;

	ssSetInputPortWidth(S, 0, 2);
	ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	ssSetInputPortWidth(S, 1, 2);
	ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
	ssSetInputPortDirectFeedThrough(S, 1, 1);

	ssSetInputPortWidth(S, 2, 3);
	ssSetInputPortRequiredContiguous(S, 2, true); /*direct input signal access*/
	ssSetInputPortDirectFeedThrough(S, 2, 1);

	if (!ssSetNumOutputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, MAX_LIST_ELEMENTS);
	ssSetOutputPortWidth(S, 1, MAX_LIST_ELEMENTS);

	ssSetNumSampleTimes(S, 1);
	ssSetNumRWork(S, 0);
	ssSetNumIWork(S, 0);
	ssSetNumPWork(S, 1);
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
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
	//ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
	ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
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

typedef struct WorkData {
	int waypoints[MAX_LIST_ELEMENTS][2];
	int start[2];
	int goal[2];
	int wall[3];
	int run;
} WorkDataType;

//-------------------------------------------------------------------
// Helper-Func für Astern inkl. astern()
//-------------------------------------------------------------------

struct search_s;
typedef struct search_s {
	struct search_s *next;
	double g;
	double h;
	int currX;
	int currY;
	int isChecked;
	int isEmpty;
} search_t;
// Struktur für Openlist
typedef struct {
	int numElements;
	search_t *firstelement;
} list_t;
// Funktionsdeklaration
void astern(int start[], int goal[], SimStruct *S);
void setWall(int x, int y, int set);
void putList(list_t *list_p, search_t *search_p);
void printList(list_t *list_p);
void printGrid(search_t GRID[MAXX][MAXY]);
void delList(list_t *list_p, int X, int Y);
search_t *getListNr(list_t *list_p, int elementnr);
void InitializeGrid(search_t Grid[MAXX][MAXY], int goal[2]);

void astern(int start[], int goal[], SimStruct *S) {
	WorkDataType * data = (const WorkDataType*)(*ssGetPWork(S));
	search_t *initialPos_p = NULL;
	search_t *goalPos_p = NULL;
	search_t *element1_p = NULL;
	search_t *element2_p = NULL;
	list_t openList_p = { 0,NULL };
	search_t expandNode = { 0,0,0,0,FALSE,FALSE };
	search_t runCell = { 0,0,0,0,FALSE,FALSE };
	int Found = FALSE;
	int Resign = FALSE;
	search_t Grid[MAXX][MAXY] = { 0 };
	int ActionTaken[MAXX][MAXY] = { 0 };
#ifdef PRINTGRID
	// Array zur Visualisierung
	char path[MAXX][MAXY];
#endif // PRINTGRID
	/* Array für mögliche Bewegungsrichtungen

	x-----+
		-1,-1 | -1, 0 | -1, 1
	y	---------------------
	|	 0,-1 |  0, 0 |  0, 1
	|	---------------------
	+	 1,-1 |  1, 0 |  1, 1

	*/
	int delta[SIZEDELTA1][SIZEDELTA2] = { { -1,0 },{ 0,-1 },{ 1,0 },{ 0,1 } };
	double small = 0;
	int i, j, k, X, Y, x2, y2, idx[2], temp[2];
	double f = 0;
	InitializeGrid(Grid, goal);
#ifdef PRINTGRID
	printGrid(Grid);
	printf("------------ Nach Init -------------------------\n");
#endif // PRINTGRID

	//Startzelle 
	initialPos_p = &Grid[start[0] - 1][start[1] - 1];
	initialPos_p->isChecked = TRUE;
	// Erstellen der ZielCelle
	goalPos_p = &Grid[goal[0] - 1][goal[1] - 1];

	//Erstes Element auf die Openlist setzen
	putList(&openList_p, initialPos_p);

	// A-Stern
	small = initialPos_p->g + initialPos_p->h;
	i = 0;
	while (Found == FALSE || Resign == FALSE) {
		small = getListNr(&openList_p, 1)->g + getListNr(&openList_p, 1)->h + COST;
		for (i = 1; i <= openList_p.numElements; i++) {
			f = getListNr(&openList_p, i)->g + getListNr(&openList_p, i)->h;
			if (f <= small) {
				small = f;
				expandNode.currX = getListNr(&openList_p, i)->currX;
				expandNode.currY = getListNr(&openList_p, i)->currY;
				expandNode.g = getListNr(&openList_p, i)->g;
				expandNode.h = getListNr(&openList_p, i)->h;
				expandNode.isChecked = getListNr(&openList_p, i)->isChecked;
				expandNode.isEmpty = getListNr(&openList_p, i)->isEmpty;
				expandNode.next = getListNr(&openList_p, i)->next;
			}
		}
		//Element von der Openlist entfernen
		delList(&openList_p, expandNode.currX, expandNode.currY);

		for (i = 0; i < SIZEDELTA1; i++) {
			if (expandNode.currX + delta[i][0]<1 ||
				expandNode.currX + delta[i][0]>MAXX ||
				expandNode.currY + delta[i][1]<1 ||
				expandNode.currY + delta[i][1]>MAXY
				) {
				continue;
			}
			else
			{
				idx[0] = expandNode.currX + delta[i][0];
				idx[1] = expandNode.currY + delta[i][1];
				runCell = Grid[expandNode.currX - 1 + delta[i][0]][expandNode.currY - 1 + delta[i][1]];
				if (runCell.isChecked != TRUE && runCell.isEmpty != TRUE) {
					element1_p = &Grid[runCell.currX - 1][runCell.currY - 1];
					element2_p = &Grid[expandNode.currX - 1][expandNode.currY - 1];
					element1_p->g = element2_p->g + COST;
					// Als bearbeitet markieren
					element1_p->isChecked = TRUE;
					element1_p->next = NULL;
					putList(&openList_p, element1_p);
					ActionTaken[runCell.currX - 1][runCell.currY - 1] = i + 1;
				}
			}
			if (runCell.currX == goalPos_p->currX && runCell.currY == goalPos_p->currY && runCell.isEmpty != TRUE) {
				Found = TRUE;
				Resign = TRUE;
				element1_p->isChecked = TRUE;
#ifdef PRINTGRID
				printf("Weg gefunden!\n");
#endif //PRINTGRID
				break;
			}
		}
		if (openList_p.numElements == 0 && Found == FALSE) {
			Resign = TRUE;
#ifdef PRINTGRID
			printf("Keinen Weg gefunden!\n");
#endif //PRINTGRID
			break;
		}
	}

	// Weg vom Ziel zum Start bilden
	X = goal[0];
	Y = goal[1];
#ifdef PRINTGRID
	path[X - 1][Y - 1] = 'X';
#endif //PRINTGRID
	data->waypoints[0][0] = X;
	data->waypoints[0][1] = Y;
	i = 1;
	while (X != start[0] || Y != start[1]) {
		x2 = X - delta[ActionTaken[X - 1][Y - 1] - 1][0];
		y2 = Y - delta[ActionTaken[X - 1][Y - 1] - 1][1];
#ifdef PRINTGRID
		path[x2 - 1][y2 - 1] = 'X';
#endif //PRINTGRID
		data->waypoints[i][0] = x2;
		data->waypoints[i][1] = y2;
		X = x2;
		Y = y2;
		i++;
	}
	// printf("i= %d",i);
	// for (k=0;k<MAX_LIST_ELEMENTS;k++){
		// printf("x: %d y: %d \n",data->waypoints[k][0],data->waypoints[k][1]);
	// }

// Liste umsortieren sodass Startpunkt am Anfang steht	
	j = i - 1;   // j = letztes Element
	i = 0;       // i = erstes Element

	while (i < j) {
		temp[0] = data->waypoints[i][0];
		temp[1] = data->waypoints[i][1];
		data->waypoints[i][0] = data->waypoints[j][0];
		data->waypoints[i][1] = data->waypoints[j][1];
		data->waypoints[j][0] = temp[0];
		data->waypoints[j][1] = temp[1];
		i++;
		j--;
	}
#ifdef PRINTGRID
	for (i = 0; i < MAX_LIST_ELEMENTS; i++) {
		printf("x: %d y: %d \n", data->waypoints[i][0], data->waypoints[i][1]);
	}

	// Weg Visualisieren
	for (i = 0; i < MAXX; i++) {
		for (j = 0; j < MAXY; j++) {
			if (path[i][j] != 'X') path[i][j] = '0';
			if ((int)Grid[i][j].isEmpty == 1)path[i][j] = '1';
			printf("%c ", path[i][j]);
		}
		printf("\n");
	}
#endif //PRINTGRID
}
void setWall(int x, int y, int set) {

}
void InitializeGrid(search_t Grid[MAXX][MAXY], int goal[2]) {
	int i, j;
	search_t gridCell = { 0,0,0,0,FALSE,FALSE };
	//world grid initialisieren
	for (i = 0; i < MAXX; i++) {
		for (j = 0; j < MAXY; j++) {
			Grid[i][j].g = 0;
			Grid[i][j].h = sqrt(pow(i + 1 - goal[0], 2) + pow(j + 1 - goal[1], 2));
			Grid[i][j].next = NULL;
		}
	}
	/* HIER KANN DAS WORLDGRID VERÄNDERT WERDEN*/
	Grid[0][1].g = 1;
	Grid[1][1].g = 1;
	Grid[2][1].g = 1;
	Grid[3][1].g = 1;
	Grid[3][2].g = 1;
	Grid[3][4].g = 1;
	Grid[4][4].g = 1;


	for (i = 0; i < MAXX; i++) {
		for (j = 0; j < MAXY; j++) {
			if (Grid[i][j].g>0) {
				Grid[i][j].currX = i + 1;
				Grid[i][j].currY = j + 1;
				Grid[i][j].isEmpty = TRUE;

			}
			else {
				Grid[i][j].currX = i + 1;
				Grid[i][j].currY = j + 1;
				Grid[i][j].isEmpty = FALSE;
			}
		}
	}
}
void putList(list_t *list_p, search_t *search_p) {
	search_t *element_p = NULL;
	list_p->numElements++;
	//erstes Element in der Liste
	if (list_p->firstelement == NULL) {
		list_p->firstelement = search_p;
	}
	else {
		//Element an das Ende der Liste anhängen
		element_p = list_p->firstelement;
		while (element_p->next != NULL) {
			element_p = element_p->next;
		}
		element_p->next = (search_t*)malloc(sizeof(search_t));
		element_p->next = search_p;

	}
}
void printList(list_t *list_p) {
	search_t *element_p;
	printf("Anzahl Elemente in der Liste: %d\n", list_p->numElements);
	printf("X-Wert Y-Wert G      H       isChecked isEmpty\n");
	element_p = list_p->firstelement;
	printf("%6d %6d %1.4f %1.4f %9d %7d\n", element_p->currX, element_p->currY, element_p->g, element_p->h, element_p->isChecked, element_p->isEmpty);
	while (element_p->next != NULL) {
		element_p = element_p->next;
		printf("%6d %6d %1.4f %1.4f %9d %7d\n", element_p->currX, element_p->currY, element_p->g, element_p->h, element_p->isChecked, element_p->isEmpty);
	}
}
void delList(list_t *list_p, int X, int Y) {
	search_t *element_p, *element1_p;
	if (list_p->numElements > 0) {
		//Erstes ELement loeschen?
		if (list_p->firstelement->currX == X && list_p->firstelement->currY == Y) {
			element_p = list_p->firstelement->next;
			list_p->firstelement = element_p;
		}
		else {
			//Beliebiges anderes Element loeschen?
			element_p = list_p->firstelement;
			while (element_p->next != NULL) {
				element1_p = element_p->next;
				if (element1_p->currX == X && element1_p->currY == Y) {
					element_p->next = element1_p->next;
					//free(element1_p);
					break;
				}
				element_p = element1_p;
			}
		}
	}
	//Element aus der Liste loeschen
	list_p->numElements--;
}
search_t *getListNr(list_t *list_p, int elementnr) {
	search_t *element_p;
	int i;
	element_p = list_p->firstelement;
	if (elementnr > 1) {
		for (i = 1; i < elementnr; i++) {
			element_p = element_p->next;
		}
	}
	return element_p;
}
void printGrid(search_t Grid[MAXX][MAXY]) {
	int i, j;
	double test = 1;
	printf("Anzahl Elemente im Grid: %d x %d\n", MAXX, MAXY);
	//printf("X-Wert Y-Wert G      H       isChecked isEmpty\n");
	for (i = 0; i < MAXX; i++) {
		for (j = 0; j < MAXY; j++) {
			printf("%d ", (int)Grid[i][j].g);
		}
		printf("\n");
	}




}
//-------------------------------------------------------------------
//-------------------------------------------------------------------

static void mdlInitializeConditions(SimStruct *S)
{
	int i = 0;
	WorkDataType *data = malloc(sizeof(WorkDataType));
	data->start[0] = 0;
	data->start[1] = 0;
	data->goal[0] = 0;
	data->goal[1] = 0;
	data->run = FALSE;
	ssSetPWorkValue(S, 0, data);
	for (i = 0; i < MAX_LIST_ELEMENTS; i++) {
		data->waypoints[i][0] = DEFAULTWAYPOINT;
		data->waypoints[i][1] = DEFAULTWAYPOINT;
	}
}
#endif /* MDL_INITIALIZE_CONDITIONS */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	const real_T *u1 = (const real_T*)ssGetInputPortSignal(S, 0); //start
	const real_T *u2 = (const real_T*)ssGetInputPortSignal(S, 1); //goal
	const real_T *u3 = (const real_T*)ssGetInputPortSignal(S, 2); //wall
	//real_T       *y = ssGetOutputPortSignal(S,0); //waypoint
	real_T *y1 = ssGetOutputPortSignal(S, 0); //waypoint x
	real_T *y2 = ssGetOutputPortSignal(S, 1); //waypoint y
	WorkDataType * data = (const WorkDataType*)(*ssGetPWork(S));
	int i = 0;
	// Veränderung am Eingang?
	if (!data->run) {
		for (i = 0; i < 2; i++) {
			if ((int)u1[i] != (int)data->start[i]) {
				//printf("u1[%d]: %d start[%d]: %d run: %d \n",i,u1[i],i,data->start[i],data->run);
				data->run = TRUE;
			}

			if ((int)u2[i] != data->goal[i])
				data->run = TRUE;
			//if((int)u3[i]!=data->wall[i])
			//    data->run=TRUE;
		}
		// Start != Ziel ?
		if (u1[0] == u2[0] && u1[1] == u2[1]) {
			data->run = FALSE;
		}

	}
	// Neue Daten am Eingang?
	if (data->run) {
		data->start[0] = u1[0];
		data->start[1] = u1[1];
		data->goal[0] = u2[0];
		data->goal[1] = u2[1];
		astern(data->start, data->goal, S);

		// Wegpunktliste auf den Ausgang schreiben
		for (i = 0; i < MAX_LIST_ELEMENTS; i++) {
			y1[i] = (real_T)data->waypoints[i][0];
			y2[i] = (real_T)data->waypoints[i][1];
		}
		data->run = FALSE;
	}
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

	const real_T *u1 = (const real_T*)ssGetInputPortSignal(S, 0); //start
	const real_T *u2 = (const real_T*)ssGetInputPortSignal(S, 1); //goal
	const real_T *u3 = (const real_T*)ssGetInputPortSignal(S, 2); //wall
	int i;
	//übergabe des Pointers an Simulink
	WorkDataType * data = (const WorkDataType*)(*ssGetPWork(S));

	data->start[0] = u1[0];
	data->start[1] = u1[1];
	for (i = 0; i < MAX_LIST_ELEMENTS; i++) {
		data->waypoints[i][0] = DEFAULTWAYPOINT;
		data->waypoints[i][1] = DEFAULTWAYPOINT;
	}
}
#endif /* MDL_UPDATE */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
	const WorkDataType * data = (const WorkDataType*)(*ssGetPWork(S));
	free(data);
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

 /*=============================*
  * Required S-function trailer *
  *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
