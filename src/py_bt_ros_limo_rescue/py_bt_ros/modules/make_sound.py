from gtts import gTTS

print("MP3 파일 생성 중...")

tts1 = gTTS(text="발견했습니다. 저를 따라오세요.", lang='ko')
tts1.save("follow_me.mp3")

tts2 = gTTS(text="띠", lang='ko')
tts2.save("beep.mp3")

print("완료! 'follow_me.mp3'와 'beep.mp3'가 생성되었습니다.")