#ifndef MATRIX
#define MATRIX
#include <iostream>
#include <cstdlib> 
using namespace std ;

#include "Matrix.h"

Matrix::Matrix() {
    _data = new double[1] ;
    _nRows = 0 ;
    _nCols = 0 ;
}

Matrix::Matrix(int size) {
    setSize( size ) ;
}

Matrix::Matrix(int nRows, int nCols) {
    setSize( nRows, nCols ) ;
}

Matrix::Matrix(int nRows, int nCols, double *data) {
    setSize( nRows, nCols ) ;
    for ( int r = 0 ; r < nRows ; r++ ){
        for ( int c = 0 ; c < nCols ; c++ ){
            _data[ r*nCols + c ] = data[ r*nCols + c ] ;
        }
    }
}

Matrix::~Matrix() {
    delete [] _data ;
}

void Matrix::setSize( int size ) {
    _size = size ;
    _data = new double[size] ;
    int x ;
    for ( x = 0 ; x < size ; x++ ){
        _data[ x ] = 0.0 ;
    }
    _nRows = size ;
    _nCols = 1 ;
}

void Matrix::setSize( int nRows, int nCols ) {
    _size = nRows * nCols ;
    _data = new double[ _size ] ;
    int x ;
    for ( x = 0 ; x < _size ; x++ ){
        _data[ x ] = 0.0 ;
    }
    _nRows = nRows ;
    _nCols = nCols ;
}

int Matrix::getSize( ) {
    return _size ; 
}

double * Matrix::getPtr(int i) {
    return (_data + i ) ;
}

double * Matrix::getPtr( ) {
    return (_data) ;
}

double Matrix::get(int i) const {
    return _data[ i ] ;
}

double Matrix::get(int i, int j) const {
    return _data[ i*_nCols + j ] ;
}

void Matrix::set(int i, double val) {
    _data[ i ] = val ;
}

void Matrix::set(int i, int j, double val) {
    _data[ i*_nCols + j ] = val ;
}

void Matrix::add(int i, double val) {
    _data[ i ] += val ;
}

void Matrix::add(int i, int j, double val) {
    _data[ i*_nCols + j ] += val ;
}

//~ void Matrix::increaseCols() {
    //~ nCols   = _nCols + 1 ;
    //~ _size  += _nRows ;
    //~ data    = new double[ _size ] ;
    
    //~ for ( int r = 0 ; r < _nRows ; r++ ){
        //~ for ( int c = 0 ; c < _nCols ; c++ ){
            //~ data[ r*nCols + c ] = _data[ r*_nCols + c ] ;
        //~ }
        //~ data[ r*nCols + _nCols ] = 0.0 ;
    //~ }
    
    //~ delete [] _data ;
    //~ _data = data ;
    //~ _nCols = nCols
//~ }

//~ void Matrix::increaseRows() {
    //~ nRows   = _nRows + 1 ;
    //~ _size  += _nCols ;
    //~ data    = new double[ _size ] ;
    
    //~ for ( int r = 0 ; r < _nRows ; r++ ){
        //~ for ( int c = 0 ; c < _nCols ; c++ ){
            //~ data[ r*_nCols + c ] = _data[ r*_nCols + c ] ;
        //~ }
    //~ }
    //~ for ( int c = 0 ; c < _nCols ; c++ ){
        //~ data[ _nRows*_nCols + c ] = 0.0 ;
    //~ }
    
    //~ delete [] _data ;
    //~ _data = data ;
    //~ _nRows = nRows
//~ }

//~ Matrix Matrix::multiply( Matrix M, int nRows, int nCols ) {
    //~ if ( _nCols != nRows ) {
        //~ throw "Error in Matrix multiplication: _nCols != nRows." ;
    //~ }
    
    //~ int i,j,k ;
    //~ double sum ;
    
    //~ Matrix Result = new Matrix( _nRows, nCols ) ;
    
    //~ for ( i = 0 ; i < _nRows ; i++ ) {
        //~ for ( j = 0 ; j < nCols ; j++ ) {
            //~ sum = 0 ;
            
            //~ for ( k = 0 ; k < _nCols ; k++ ) {
               //~ sum += get( i, k ) * M.get( k, j ) ;
            //~ }
            
            //~ Result.set( i, j, sum ) ;
        //~ }
    //~ }
    
    //~ return Result ;
//~ }

#endif //MATRIX
