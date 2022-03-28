package bridge_types;

////////////////////////////////////////////////////////////////////////////////                    
/// Typeclass Definition                                                                            
////////////////////////////////////////////////////////////////////////////////                    
typeclass ConnectableClocks#(type a, type b);                                                       
   module mkConnectionClocks#(a x1, Clock fastClock, Reset fastReset, b x2, Clock slowClock, Reset slowReset)(Empty);
endtypeclass 

endpackage
