function hLib = fx_table


hLib = RTW.TflTable;
%---------- entry: lookupFx ----------- 
hEnt = createCRLEntry(hLib, ...
    'double y1 = lookupFx( double u1, double u2[2 2], double u3[0 0; Inf Inf], int32 u4 )', ...
    'double y1 = crl_LUT( double u1, double* u2, double* u3, int32 u4 )');
hEnt.setTflCFunctionEntryParameters( ...
          'Priority', 100, ...
          'SideEffects', true);



hLib.addEntry( hEnt ); 

