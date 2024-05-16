#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <time.h>
#include <sys/shm.h>


#define PORT 8080
#define BUFFER_SIZE 1024


struct data
{
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};

int main()
{
  struct sockaddr_in server_addr, client_addr;
  int sockfd, nbytes;
  socklen_t addr_len;
  char buffer[BUFFER_SIZE];


  // Create UDP socket
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket");
    exit(EXIT_FAILURE);
  }

  // Bind to port
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(PORT);
  if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    perror("bind");
    exit(EXIT_FAILURE);
  }

  printf("Listening for UDP datagrams on port %d...\n", PORT);


  //shared memory init
  int segment_id;
  data * shared_memory;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey = 33222;
  int ch = 0;

  /* Allocate a shared memory segment.  */
  segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (data *) shmat(segment_id, 0, 0);
  printf("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl(segment_id, IPC_STAT, &shmbuffer);
  segment_size = shmbuffer.shm_segsz;
  printf("segment size: %d\n", segment_size);


  // Receive data
  addr_len = sizeof(client_addr);
  while (1) {
    nbytes = recvfrom(
      sockfd, (char *)buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr,
      &addr_len);
    buffer[nbytes] = '\0';
    if (nbytes < 0) {
      perror("recvfrom");
      exit(EXIT_FAILURE);
    }

    shared_memory->key0 = buffer[4];
    shared_memory->key1 = buffer[5];
    shared_memory->key2 = buffer[6];
    shared_memory->key3 = buffer[7];
    shared_memory->thrust = buffer[1];
    shared_memory->yaw = buffer[0];
    shared_memory->pitch = buffer[3];
    shared_memory->roll = buffer[2];
    shared_memory->sequence_num = buffer[8];

    // printf(
    //   "Received %d bytes from %s:%d\n", nbytes, inet_ntoa(client_addr.sin_addr),
    //   ntohs(client_addr.sin_port));
    // //printf("Message: %s\n", nbytes);
    // printf("byte 0 is %d\n", buffer[0]);
    // printf("byte 1 is %d\n", buffer[1]);
    // printf("byte 2 is %d\n", buffer[2]);
    // printf("byte 3 is %d\n", buffer[3]);
    // printf("byte 4 is %d\n", buffer[4]);
    // printf("byte 5 is %d\n", buffer[5]);
    // printf("byte 6 is %d\n", buffer[6]);
    // printf("byte 7 is %d\n", buffer[7]);
    // printf("byte 8 is %d\n", buffer[8]);
  }

  // Close socket
  // close(sockfd);

  return 0;
}
