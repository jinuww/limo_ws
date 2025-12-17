from gtts import gTTS

# 1. 원하는 멘트 입력
text = "비상구에 도착했습니다. 안전하게 대피하세요."

# 2. 한국어로 변환
tts = gTTS(text=text, lang='ko')

# 3. 파일 저장
tts.save("arrived_exit.mp3")

print("arrived_exit.mp3 생성 완료!")