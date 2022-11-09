#ifndef PUBSUB_H_
#define PUBSUB_H_

#include <stdbool.h>
#include "ll.h"
#include "semphr.h"



typedef void* (*Job)(void* params);

/**
 * The Publisher type.
 * @field id The string id of this Publisher
  */
typedef struct Publisher {
    uint8_t id;
    uint8_t len;
    QueueHandle_t queueHandler;
    Item* ll_sub;
} Publisher;


typedef struct PullPublisher {
    uint8_t id;
    uint32_t rate;
    Job job;
} PullPublisher;

/**
 * The Subscriber type.
  */
typedef struct Subscriber {
    QueueHandle_t queueHandler;
    TaskHandle_t taskHandle;
} Subscriber;


void pubsubInit();
bool pubsubTest();


uint8_t registerPublisher(uint8_t id, uint8_t len,  QueueHandle_t queueHandler);
uint8_t registerPublisherPull(uint8_t id, uint8_t len, QueueHandle_t queueHandler, uint32_t rate, Job job);
bool registerSubscriber(uint32_t publisherId, TaskHandle_t taskHandle, QueueHandle_t queueHandler);

void publish(int publisherIndex);

#endif /* PUBSUB_H_ */
