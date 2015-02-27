#ifndef CNEURALNETWORK
#define CNEURALNETWORK
#define THRESHOLD 2
#define TANH 1
#define LINEAR 0
# include <cstdlib>
# include <iostream>
# include <math.h>
# include "cNeuralNetwork.h"
# include "cFunction.h"
# include "cLinear.cpp"
# include "cThreshold.cpp"
# include "cTanH.cpp"
# include "Matrix.cpp"

using namespace std ;

cNeuralNetwork::cNeuralNetwork( const char * nnFile ) {
    SIZEOFDOUBLE = sizeof( double ) ;
    readNetwork( nnFile ) ;
}

cNeuralNetwork::cNeuralNetwork( int nLayersInit, int * layerSizeInit ) {
    int * layerFunctions = new int[ nLayersInit + 2 ] ;
    for ( int l = 1 ; l < nLayersInit + 1 ; l++ ) {
        layerFunctions[ l ] = TANH ;
    }
    layerFunctions[ 0 ] = LINEAR ;
    layerFunctions[ nLayersInit + 1 ] = LINEAR ;
    
    init( nLayersInit, layerSizeInit, layerFunctions ) ;
    
    delete [] layerFunctions ;
    
}
    
cNeuralNetwork::cNeuralNetwork( int nLayersInit, int * layerSizeInit, int * layerFunctionInit ) {
    // int nLayersInit : the number of hidden layers (So for input, hidden, output -> 1)
    // int * layerSizeInit : the number of nodes per layer (# nodes layers = # weights layers + 1)
    //                       (size should be nLayersInit + 2)
    // int * layerFunctionInit : code, specifying the function per (node) layer

    //~ cout << "constructor NN L\n" ;
    init( nLayersInit, layerSizeInit, layerFunctionInit ) ;
}

void cNeuralNetwork::init( int nLayersInit, int * layerSizeInit, int * layerFunctionInit ) {
    srand48(time(0));
    
    SIZEOFDOUBLE = sizeof( double ) ;
    
    nLayers = nLayersInit + 2 ; // # node layers == # hidden layers + 2
    nInput  = layerSizeInit[ 0 ] ;
    nOutput = layerSizeInit[ nLayers - 1 ] ;
    
    //~ cout << "nInput" << nInput << endl ;
    //~ cout << "nOutput" << nOutput << endl ;
    //~ cout << "nLayers" << nLayers << endl ;
    
    layerFunction       = new cFunction*[ nLayers ] ;
    layerFunctionInts   = new int[ nLayers ] ;
    weights             = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
    layerIn             = new Matrix*[ nLayers ] ; 
    layerOut            = new Matrix*[ nLayers ] ;
    layerSize           = new int[ nLayers ] ;
    
    for( l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l + 1 ] ) ; // layerSize[ l ] + 1, for bias
    }
    for( l = 0 ; l < nLayers ; l++ ) {
        layerSize[ l ] = layerSizeInit[ l ] ;
        layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
        layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
        
        layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
        if ( layerFunctionInit[ l ] == TANH ) {
            layerFunction[ l ] = new cTanH() ;
        } else if ( layerFunctionInit[ l ] == LINEAR ) {
            layerFunction[ l ] = new cLinear() ;
        } else {
            cout << "WARNING: Unknown layer function type: " ;
            cout << layerFunctionInit[ l ] << '\n' ;
            cout << "layer: " << l << '\n' ;
            exit(1) ;
        }
    }
    
    // Initialise the weights randomly between -0.3 and 0.3
    randomizeWeights( -0.3, 0.3 ) ;
    recentlyUpdated = true ;
}

cNeuralNetwork::~cNeuralNetwork( ) {
    //~ cout << "destructor NN\n" ;
    for( l = 0 ; l < nLayers - 1 ; l++ ) {
        delete weights[ l ] ;
    }
    for( l = 0 ; l < nLayers ; l++ ) {
        delete layerIn[ l ] ;
        delete layerOut[ l ] ;
        delete layerFunction[ l ] ;
    }
    delete [] layerIn ;
    delete [] layerOut ;
    delete [] layerFunction ;
    delete [] layerFunctionInts ;
    delete [] layerSize ;
    delete [] weights ;
}

void        cNeuralNetwork::changeFunction( int layer, int function ) {
    delete layerFunction[ layer ] ;
    
    if ( function == TANH ) {
        
        layerFunction[ layer ] = new cTanH() ;
    
    } else if ( function == LINEAR ) {
        
        layerFunction[ layer ] = new cLinear() ;
    
    } else if ( function == THRESHOLD ) {
        
        layerFunction[ layer ] = new cThreshold() ;
    
    }
    
    recentlyUpdated = true ;
}
    
void        cNeuralNetwork::_forwardPropLayer( int layer ) {

    w = weights[ layer ]->getPtr() ; // The weights of the current layer
    
    nFormer = layerSize[ layer ] ;       // # nodes in the former nodes layer
    nNext   = layerSize[ layer + 1 ] ;   // # nodes in the next nodes layer
    
    formerOut  = layerOut[ layer ]->getPtr() ;      // The output of the former layer
    nextIn     = layerIn[ layer + 1 ]->getPtr() ;   // The input of the next layer (to be set)
    nextOut    = layerOut[ layer + 1 ]->getPtr() ;  // The output of the next layer (follows from the input and function)
    
    cFunction * f = layerFunction[layer+1] ;                 // The function on the next layer

    // Initialise inputs to the next layer to zero
    for( o = 0 ; o < nNext ; o++ ) {
        nextIn[ o ] = 0.0 ;
    }
    
    // initialise counter for the weights
    int wPos = 0 ;
    // add weighted inputs
    for( i = 0 ; i < nFormer ; i++ ) {
        if ( formerOut[ i ] != 0.0) {
            for( o = 0 ; o < nNext ; o++ ) {
                nextIn[ o ] += w[ wPos ] * formerOut[ i ] ;
                wPos++ ;
            }
        } else {
            wPos += nNext ;
        }
    }
    // add bias and calculate output
    for( o = 0 ; o < nNext ; o++ ) {
        nextIn[ o ] += w[ wPos ] ; 
        wPos++ ;
    }
    f->output( nextIn, nextOut, nNext ) ;
}

double *    cNeuralNetwork::_backPropLayer( int layer, double * oError, double learningSpeed ) {
    
    w = weights[ layer ]->getPtr() ; // The weights of the current layer
    
    nFormer = layerSize[ layer ] ;   // # nodes in the former nodes layer
    nNext   = layerSize[ layer + 1 ] ;       // # nodes in the next nodes layer
    
    formerOut = layerOut[ layer ]->getPtr() ; // The output of the former layer
    formerIn  = layerIn[ layer ]->getPtr() ;  // The input of the former layer
    
    cFunction * f = layerFunction[ layer ] ; // Function of the former layer
    
    wPos = 0 ;
    
    iError = new double[ nFormer ] ;   // Error of the input of the former layer
    
    if ( layer > 0 ) {
        
        for( i = 0 ; i < nFormer ; i++ ) {
            
            iError[ i ] = 0.0 ;
            
            if ( formerOut[ i ] != 0.0 ) {
                
                for ( o = 0 ; o < nNext ; o++ ) {
                    
                    // First get error:
                    iError[ i ] += w[ wPos ] * oError[ o ] ;
                    
                    // Then update the weight:
                    w[ wPos ] += formerOut[ i ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            } else {
                
                for ( o = 0 ; o < nNext ; o++ ) {
                    
                    // Only get error:
                    iError[ i ] += w[ wPos ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            }
            // Pass the error through the layers function:
            iError[ i ] *= f->derivative( formerIn[ i ], formerOut[ i ] ) ;
            
        }
        
    } else {
        
        for( i = 0 ; i < nFormer ; i++ ) {
            
            if ( formerOut[ i ] != 0.0 ) {
                
                for ( o = 0 ; o < nNext ; o++ ) {
                   
                    // Only update weight:
                    w[ wPos ] += formerOut[ i ] * oError[ o ] ;
                    
                    wPos++ ;
                    
                }
                
            } else {
                
                wPos += nNext ;
                
            }
            
        }
    }

    for ( o = 0 ; o < nNext ; o++ ) {
        
        // Update the bias
        w[ wPos ] += oError[ o ] ;
        
        wPos++ ;
        
    }
    
    
    
    return iError ;
}

void        cNeuralNetwork::backPropagate( double *input, double *target, double learningSpeed ) {
    //If the network has been adapted after the last forward propagation,
    //forwardPropagate first to correctly set the hidden activations:
    if ( recentlyUpdated ) {
        forwardPropagate( input ) ;
    } else {
        //Check whether the activations of the layers correspond with the present input
        inputIn = layerIn[ 0 ]->getPtr() ;
        bool useLastActivations = true ;
        for ( i = 0 ; ( i < nInput ) & useLastActivations ; i++ ){
            if ( input[i] != inputIn[i] ) {
                useLastActivations = false ;
            }
        }
        if ( !useLastActivations ) {
            //If the activations don't correspond to the last input (and the network
            //has been adapted in the meantime) set the activations by a forward
            //propagation
            forwardPropagate( input ) ;
        }
    }
    error = new double[ nOutput ] ; //error of the output layer.
    outputOut  = layerOut[ nLayers - 1 ]->getPtr() ;// Output of the output layer
    
    for( o = 0 ; o < nOutput ; o++) {
        error[ o ] = target[ o ] - outputOut[ o ] ;
    }
    //backPropagate the error
    backPropagateError( input, error, learningSpeed ) ;
    delete[] error ;
}

void        cNeuralNetwork::backPropagateError( double *input, double *error, double learningSpeed ) {
    //If the network has been adapted after the last forward propagation,
    //forwardPropagate first to correctly set the hidden activations:
    if ( recentlyUpdated ) {
        forwardPropagate( input ) ;
    } else {
        //Check whether the activations of the layers correspond with the present input
        inputIn = layerIn[ 0 ]->getPtr() ;
        bool useLastActivations = true ;
        for ( i = 0 ; ( i < nInput ) & useLastActivations ; i++ ){
            if ( input[i] != inputIn[i] ) {
                useLastActivations = false ;
            }
        }
        if ( !useLastActivations ) {
            //If the activations don't correspond to the last input (and the network
            //has been adapted in the meantime) set the activations by a forward
            //propagation
            forwardPropagate( input ) ;
        }
    }
    oError = new double[ nOutput ] ; //error of the output layer.
    
    outputIn   = layerIn[ nLayers - 1 ]->getPtr() ; // Input of the output layer
    outputOut  = layerOut[ nLayers - 1 ]->getPtr() ;// Output of the output layer
    cFunction * f       = layerFunction[ nLayers - 1 ] ;    // Function of the output layer

    //First calculate the error of the input of the output layer:
    for( o = 0 ; o < nOutput ; o++) {
        oError[ o ] = learningSpeed*error[ o ] ;
        oError[ o ] *= f->derivative( outputIn[ o ], outputOut[ o ] ) ;
    }
    
    //Now propagate until reaching the input layer:
    for ( l = nLayers - 2 ; l >= 0 ; l-- ) {
        iError = _backPropLayer( l, oError, learningSpeed ) ;
        delete [] oError ;
        oError = iError ;
    }
    
    delete [] iError ;
    
    recentlyUpdated = true ;
}
    


double *    cNeuralNetwork::forwardPropagate( double *input ) {
    cFunction * f       = layerFunction[ 0 ] ; // Function of the input layer
    
    double * inputIn  = layerIn[ 0 ]->getPtr() ;  // Input of the first layer (to be set)
    double * inputOut = layerOut[ 0 ]->getPtr() ; // Output of the first layer (to be set)
    
    //First set the first layer
    for( i = 0 ; i < nInput ; i++ ) {
        inputIn[ i ] = input[ i ] ;
    }
    
    f->output( input, inputOut, nInput ) ;
    
    //Now propagate (sets layerIn and layerOut for each layer)
    for ( l = 0 ; l < (nLayers - 1) ; l++ ) {
        _forwardPropLayer( l ) ;
    }
    
    //Set recentlyUpdated to false to show that the activations where set after the last change to the network
    recentlyUpdated = false ;
    
    return layerOut[ nLayers - 1 ]->getPtr() ; // Output of the last layer

}

double *    cNeuralNetwork::forwardPropagate( vector<double> *input_vector ) {
    if ( (int) input_vector->size() != nInput ) {
        cerr << "Vector input is incorrect size: " << input_vector->size() << " instead of " << nInput << endl ;
        throw -1 ;
    }
    
    double * input = new double[ nInput ] ;
    for ( i = 0 ; i < nInput ; i++ ) {
        input[ i ] = input_vector->at( i ) ;
    }
    
    double * output = forwardPropagate( input ) ;
    
    delete [] input ;
    
    return output ;
}
    
    
void        cNeuralNetwork::forwardPropagate( double *input, List * output ) {
    cFunction * f       = layerFunction[ 0 ] ; // Function of the input layer
    
    double * inputIn  = layerIn[ 0 ]->getPtr() ;  // Input of the first layer (to be set)
    double * inputOut = layerOut[ 0 ]->getPtr() ; // Output of the first layer (to be set)
    
    //~ cout << "nInput" << nInput << endl ;
    //~ cout << "nOutput" << nOutput << endl ;
    //~ cout << "nLayers" << nLayers << endl ;
    
    //First set the first layer
    for( i = 0 ; i < nInput ; i++ ) {
        inputIn[ i ] = input[ i ] ;
    }
    
    f->output( input, inputOut, nInput ) ;
    
    //Now propagate (sets layerIn and layerOut for each layer)
    for ( l = 0 ; l < (nLayers - 1) ; l++ ) {
        _forwardPropLayer( l ) ;
    }

    double * outputOut = layerOut[ nLayers - 1 ]->getPtr() ; // Output of the last layer
    
    //Finally, return the output of the last layer through the argument
    output->contents = new double[ nOutput ] ;
    for ( o = 0 ; o < nOutput ; o++ ) {
        output->contents[ o ] = outputOut[ o ] ;
    }
    output->length = nOutput ;
    //Set recentlyUpdated to false to show that the activations where set after the last change to the network
    recentlyUpdated = false ;
}

double      cNeuralNetwork::getActivation( int layer, int node )  {
    return layerOut[ layer ]->get( node ) ;
}

void        cNeuralNetwork::getActivations( int layer, List * activations )  {
    double * layerActivation = layerOut[ layer ]->getPtr() ; // Output of the requested layer
    
    //Return the output of the requested layer through the argument
    int nActivations = layerSize[ layer ] ;
    activations->contents = new double[ nActivations ] ;
    for ( o = 0 ; o < nActivations ; o++ ) {
        activations->contents[ o ] = layerActivation[ o ] ;
    }
    activations->length = nActivations ;
}

double *    cNeuralNetwork::getActivations( int layer )  {
    return layerOut[ layer ]->getPtr() ; // Output of the requested layer
}

double      cNeuralNetwork::getWeights( int layer, int i, int j )  {
    return weights[ layer ]->get( i*(layerSize[ layer + 1 ]) + j ) ;
}

void        cNeuralNetwork::setWeights( int layer, int i, int j, double val ) {
    recentlyUpdated = true ;
    weights[ layer ]->set( i*(layerSize[ layer + 1 ]) + j, val ) ;
}

void        cNeuralNetwork::randomizeWeights( double min, double max ) {
    for ( l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
                setWeights( l, i, o, min + (max - min)*drand48() ) ;
            }
        }
    }
}

void        cNeuralNetwork::randomizeWeights( double min, double max, int seed ) {
    srand( seed ) ;
    for ( l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
                setWeights( l, i, o, min + (max - min)*drand48() ) ;
            }
        }
    }
}

void        cNeuralNetwork::printNetwork() {
    for ( l = 0 ; l < (nLayers - 1) ; l++ ) {
        _printLayer( l ) ;
    }
}

void        cNeuralNetwork::_printLayer( int layer ) {
    double * w = weights[ layer ]->getPtr() ; // The weights of the current layer
    cout << "layer " << layer ;
    
    int nFormer = layerSize[ layer ] ;       // # nodes in the former nodes layer
    int nNext   = layerSize[ layer + 1 ] ;   // # nodes in the next nodes layer
    
    cout << " nFormer " << nFormer ;
    cout << " nNext " << nNext << endl ;
    
    int wPos = 0 ;
    for( i = 0 ; i < nFormer ; i++ ) {
        for( o = 0 ; o < nNext ; o++ ) {
            cout << w[ wPos ] << ", " ;
            wPos++ ;
        }
        cout << '\n' ;
    }
    // add bias and calculate output
    for( o = 0 ; o < nNext ; o++ ) {
        cout << " B " <<  w[ wPos ] << '\n' ; 
        wPos++ ;
    }
}

int         cNeuralNetwork::read_int(istream &is) {
    int result;
    char *s = (char *) &result;
    is.read(s, sizeof(result));
    return result;
}

void        cNeuralNetwork::write_int(ostream &is, int result) {
    char *s = (char *) &result;
    is.write(s, sizeof(result));
}

double      cNeuralNetwork::read_double(istream &is) {
    double result;
    char *s = (char *) &result;
    is.read(s, sizeof(result));
    return result;
}

void        cNeuralNetwork::write_double(ostream &is, double result) {
    char *s = (char *) &result;
    is.write(s, sizeof(result));
}

void        cNeuralNetwork::readNetwork( const char * file ) { 
    ifstream ifile ;
    
    ifile.open( file, ifstream::in ) ;
    
    nLayers                 = read_int( ifile ) ; 
    
    int * layerSizeInit     = new int[nLayers] ;
    int * layerFunctionInit = new int[nLayers] ;
    
    for ( int l = 0 ; l < nLayers ; l++ ) {
        layerSizeInit[l]        = read_int( ifile ) ; 
        layerFunctionInit[l]    = read_int( ifile ) ;
    }

    nInput  = layerSizeInit[ 0 ] ;
    nOutput = layerSizeInit[ nLayers - 1 ] ;
    //~ cout << "nInput" << nInput << endl ;
    //~ cout << "nOutput" << nOutput << endl ;
    //~ cout << "nLayers" << nLayers << endl ;
    
    layerFunction = new cFunction*[ nLayers ] ;
    layerFunctionInts   = new int[ nLayers ] ;
    weights       = new Matrix*[ nLayers - 1 ] ; // # weights layers = # node layers - 1
    layerIn       = new Matrix*[ nLayers ] ; 
    layerOut      = new Matrix*[ nLayers ] ;
    layerSize     = new int[ nLayers ] ;
    
    for( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        weights[ l ]  = new Matrix( layerSizeInit[ l ] + 1, layerSizeInit[ l + 1 ] ) ; // layerSize[ l ] + 1, for bias
    }
    for( int l = 0 ; l < nLayers ; l++ ) {
        layerSize[ l ] = layerSizeInit[ l ] ;
        layerIn[ l ]  = new Matrix( layerSize[ l ] ) ;
        layerOut[ l ] = new Matrix( layerSize[ l ] ) ;
            
        layerFunctionInts[ l ] = layerFunctionInit[ l ] ;
        if ( layerFunctionInit[ l ] == TANH ) {
            layerFunction[ l ] = new cTanH() ;
        } else if ( layerFunctionInit[ l ] == LINEAR ) {
            layerFunction[ l ] = new cLinear() ;
        } else {
            cout << "WARNING: Unknown layer function type: " ;
            cout << layerFunctionInit[ l ] << '\n' ;
            cout << "layer: " << l << '\n' ;
            exit(1) ;
        }
    }
    // Get weights
    for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
                setWeights( l, i, o, read_double( ifile ) ) ;
            }
        }
    }
    recentlyUpdated = true ;
    
    delete [] layerSizeInit ;
    delete [] layerFunctionInit ;
}
    
void        cNeuralNetwork::writeNetwork( const char * file ) { 
    ofstream ofile ;
    
    ofile.open( file, ofstream::out ) ;
    
    write_int( ofile, nLayers ) ; 
    
    for ( int l = 0 ; l < nLayers ; l++ ) {
        write_int( ofile, layerSize[l] ) ; 
        write_int( ofile, layerFunctionInts[l] ) ;
    }

    // Weights
    for ( int l = 0 ; l < ( nLayers - 1 ) ; l++ ) {
        for ( int i = 0 ; i < ( layerSize[l] + 1 ) ; i++ ) {
            for ( int o = 0 ; o < layerSize[ l + 1 ] ; o++ ) {
                write_double( ofile, getWeights( l, i, o ) ) ;
            }
        }
    }
}

#endif //CNEURALNETWORK

