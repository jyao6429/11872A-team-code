#ifndef INDEXER_H
#define INDEXER_H

namespace indexer
{
    /** 
     * Moves indexer with target voltage while insuring safety with a mutex
     *
     * @param indexerVolt - desired voltage for the indexer motor
     */
    void moveVoltageSafe(int indexerVolt);
    /** 
     * Moves indexer with target voltage
     *
     * @param indexerVolt - desired voltage for the indexer motor
     */
    void moveVoltage(int indexerVolt);
    /**
    * Fucntion for indexer control during driver
    */
    void opcontrol();
}

#endif