String int2string(int num){
  String result;
  int leng=count(num);
  int prv;//holds digits before one to be conveted
  for(int i=0;i<leng;i++){
    prv=num/pow(10,leng-i);
    result=result+dig2char((num/pow(10,(leng-1-i)))-(prv*10));
  }
  return result;
}
String float2string(float num){
  int unit=num;
  int dec=(num-unit)*101;//101 for acccurate calculations
  String result;
  if(unit==0)result+='0';
  else result=int2string(unit);
  result=result+dig2char(46);//to insert a period
  if(dec<10)
  {
    result=result+'0'+dig2char(dec);
  }
  else{
    result=result+int2string(dec);
  }
  return result;
}
int count(int num){
  byte reverse=0;
  for(int i=num;i!=0;i/=10){
    reverse++;
  }
  return reverse;
}
char dig2char(byte num){
  if(num==0)
  return '0';
  else if(num==1)
  return '1';
  else if(num==2)
  return '2';
  else if(num==3)
  return '3';
  else if(num==4)
  return '4';
  else if(num==5)
  return '5';
  else if(num==6)
  return '6';
  else if(num==7)
  return '7';
  else if(num==8)
  return '8';
  else if(num==9)
  return '9';
  else if(num==46)
  return '.';
}
