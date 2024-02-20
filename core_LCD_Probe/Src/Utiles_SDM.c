// Biblioteca de funciones auxiliares

void espera(int tiempo) {
   int i;
   for (i=0; i<tiempo; i++);
   }

void Bin2Ascii(signed short numero, unsigned char* cadena){
   
	signed short parcial, cociente, divisor;
	unsigned short i;
	
  if (numero<0)
	{
		numero = numero*(-1);
		*(cadena)='-';
	}
  else
	{
		*(cadena)='0';
	}
	parcial = numero;
  divisor = 10000;
	
	for (i=1; i<6; i++){
      cociente = parcial/divisor;
      *(cadena+i) = '0'+(unsigned char)cociente;
      parcial = parcial - (cociente * divisor);
      divisor = divisor / 10;
      }
}
