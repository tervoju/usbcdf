#include "src\lib_common.h"
#ifdef USE_CTD

//#include "src\util\AQLSalinityCalculations.h"
//#include "src\diver\diving_engine.h"

#define CTD_TIMEOUT  200
#define CTD_ACQUISITION_TRIGGER_PACKET "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",0xff,0xff,0xff,0xff,0xff,0xaa,0,0x90,0,0,0,0,0,0,0x6c
/*	Sending this packet will trigger an acquisition cycle and the logger will
 *	output an NMEA string with the current reading set.  The acquisition will
 *	take approximately 1.5 seconds before the output string is returned.
 *
 *	The data format for input & output is RS232 8:1:N at 9600 baud.
 *
 *	NMEA output string:
 *	  $AQCTD,<Temperature>,<Pressure>,<Conductivity>*<Checksum><CR><LF>
 *	         [(-)2.3]°C (IPTS90)
 *	                       [(-)2.3] bar (absolute)
 *	                                  [(-)2.3] mScm^-1.
 *
 *	Take 1 off the absolute pressure (to compensate for atmospheric pressure)
 *	and multiply by 10 for a rough bar to m water depth conversion.
 *
 *	shallow freshwater depth approximation: depth = gauge pressure * 1.019716
 */

volatile CTDDataStruct ctd;


/*	-----------------------------------------------------------------------------
 *	Depth in meters from pressure in decibars using Saunders and Fofonoff's
 *	method from UNESCO Technical Papers in Marine Science, Vol. 44, 1983
 *	Deep-sea Res., 1976,23,109-111
 *	using standard ocean: T = 0 °C, S = 35 (PSS-78)
 *
 *	FIXME: needs to be adjusted for Baltic standards: non-zero T, lower S
 */
float calc_depth( float pres, float lat ) {
	float specific_volume, s, gravity_variation;

	// FIXME: actually measure & record atmospheric surface pressure
	pres = ( pres - 1.101325 ) * 10; // from bar absolute pressure to decibar gauge pressure
	if ( pres < 0 ) return 0;

	// FIXME: assume much lower salinity
	specific_volume = pres * ( 9.72659 + pres * ( -2.2512e-5 + pres * ( 2.279e-10 + pres * -1.82e-15 ) ) );

	s = sin( lat / 57.29578 );
	s *= s; // s = sin(lat°) squared
	// 2.184e-6 is mean vertical gradient of gravity in m/s^2/decibar
	gravity_variation = 9.780318 * ( 1.0 + s * ( 5.2788e-3 + s * 2.36e-5 ) ) + 1.096e-6 * pres;

	return specific_volume / gravity_variation;	// depth in meters
}


//-----------------------------------------------------------------------------
BOOLEAN ctd_read_values( unsigned char * p, CTDDataStruct * c ) {
	p = strstr(p,"$AQCTD"); if (!p) return FALSE;
	if ( !nmea_verify_checksum(p) ) return FALSE;

	p = strchr(p,','); if (!p) return FALSE;
	c->temperature = (long)( atof(++p) * 1000 );

	p = strchr(p,','); if (!p) return FALSE;
	c->pressure = (long)( atof(++p) * 1000 );

	p = strchr(p,','); if (!p) return FALSE;
	c->conductivity = (long)( atof(++p) * 1000 );

	return TRUE;
}


//-----------------------------------------------------------------------------
static void ctd_cycle (void) _task_ TASK_CTD _priority_ PRIORITY_CTD {
	t_rtx_exit s;

	ctd.status = LOG_CODE_INVALID;

	GET_SERIAL_TOKEN;
		rx_write = RX_WRITE_INIT;
		serial_switch_port(SERIAL_CTD_PORT,1);
		printf(CTD_ACQUISITION_TRIGGER_PACKET);

		s = os_wait_signal(CTD_TIMEOUT);
	FREE_SERIAL_TOKEN;

	if ( s != SIG_EVENT ) {
		ctd.status = LOG_CODE_SILENCE;
		log_write_status( LOG_SOURCE_CTD, 0, TRUE );
		END_TASK;
	}

	for(;;) {
		os_wait_signal(FOREVER);

		GET_SERIAL_TOKEN;
			serial_stop_rx_buffer_write(0);
			serial_switch_port(SERIAL_CTD_PORT,1);
			rx_write = RX_WRITE_INIT;
			printf(CTD_ACQUISITION_TRIGGER_PACKET);
			s = os_wait_signal(CTD_TIMEOUT);
			os_wait_token( ctd_token, 1, FOREVER );
				if ( s != SIG_EVENT ) ctd.status = LOG_CODE_SILENCE;
				else ctd.status = ctd_read_values( rx_buf, &ctd ) ? LOG_CODE_OK : LOG_CODE_INVALID;
			os_send_token( ctd_token, 1 );
		FREE_SERIAL_TOKEN;

		if ( ctd.status == LOG_CODE_OK ) {
			ctd.timestamp = rtc_update(FALSE);
			log_write_ctd_data( &ctd, TRUE );
			//log_write_msg( LOG_FORMAT_CHAR | LOG_MSG_STATUS, ctd_buf, 0 );
		} else {
			log_write_status( LOG_SOURCE_CTD, 0, TRUE );
			//log_write_msg( LOG_FORMAT_CHAR | LOG_MSG_ERROR, ctd_buf, 0 );
		}

		//AQLCalculateSalinity( cond, temp, pres, &salin );
		//AQLCalculateSpeedOfSound( SPEEDOFSOUNDMETHOD_CHENMILLERO, salin, temp, pres, &speed );
	}

	END_TASK;
}

#endif //USE_CTD

/*
	#ifdef USE_CTD_SIMULATOR
		replace CTD_ACQUISITION_TRIGGER_PACKET with the following:
		unsigned char * floatPtr = (unsigned char*)&Volume;
		printf("%c%c%c%c%c%c", 0x55, floatPtr[0],floatPtr[1],floatPtr[2],floatPtr[3], 0x66);
	#endif
*/