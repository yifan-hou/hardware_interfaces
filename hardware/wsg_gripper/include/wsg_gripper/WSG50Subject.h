#pragma once

class WSG50Observer;

class WSG50Subject
{
public:
    // attach a new observer to be notified
    void Attach(WSG50Observer * observer);

protected:
    WSG50Observer * _observer;
};
