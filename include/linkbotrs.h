
#ifdef __cplusplus
extern "C" {
#endif

struct Linkbot;
Linkbot* linkbot_new(const char* serial_id);
void linkbot_move(Linkbot* linkbot, float angle1, float angle2, float angle3);
void linkbot_move_nb(Linkbot* linkbot, float angle1, float angle2, float angle3);
void linkbot_move_wait(Linkbot* linkbot);

#ifdef __cplusplus
}
#endif
