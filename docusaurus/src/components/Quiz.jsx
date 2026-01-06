import React, { useState } from 'react';
import styles from './Quiz.module.css';

const Quiz = ({ questions }) => {
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedOption, setSelectedOption] = useState(null);
  const [isAnswered, setIsAnswered] = useState(false);
  const [score, setScore] = useState(0);

  const currentQuestion = questions[currentQuestionIndex];

  const handleOptionClick = (optionId) => {
    if (isAnswered) return;
    setSelectedOption(optionId);
    setIsAnswered(true);
    if (optionId === currentQuestion.correctAnswer) {
      setScore(score + 1);
    }
  };

  const handleNext = () => {
    setSelectedOption(null);
    setIsAnswered(false);
    if (currentQuestionIndex < questions.length - 1) {
      setCurrentQuestionIndex(currentQuestionIndex + 1);
    }
  };

    const handlePrevious = () => {
        setSelectedOption(null);
        setIsAnswered(false);
        if (currentQuestionIndex > 0) {
            setCurrentQuestionIndex(currentQuestionIndex - 1);
        }
    };

  const isLastQuestion = currentQuestionIndex === questions.length - 1;

  return (
    <div className={styles.quizContainer}>
      <div className={styles.questionHeader}>
        <h3>Question {currentQuestionIndex + 1} of {questions.length}</h3>
        <p>{currentQuestion.question}</p>
      </div>
      <div className={styles.optionsContainer}>
        {currentQuestion.options.map((option) => {
          const isCorrect = option.id === currentQuestion.correctAnswer;
          const isSelected = option.id === selectedOption;
          let optionClass = styles.option;
          if (isAnswered) {
            if (isCorrect) {
              optionClass = `${styles.option} ${styles.correct}`;
            } else if (isSelected) {
              optionClass = `${styles.option} ${styles.incorrect}`;
            }
          }
          return (
            <button
              key={option.id}
              className={optionClass}
              onClick={() => handleOptionClick(option.id)}
              disabled={isAnswered}
            >
              <span className={styles.optionId}>{option.id}</span>
              <span className={styles.optionText}>{option.text}</span>
            </button>
          );
        })}
      </div>
      {isAnswered && (
        <div className={styles.explanationContainer}>
          <h4>Explanation</h4>
          <p>{currentQuestion.explanation}</p>
        </div>
      )}
      <div className={styles.navigation}>
        <button onClick={handlePrevious} disabled={currentQuestionIndex === 0}>Back</button>
        {!isLastQuestion && <button onClick={handleNext} disabled={!isAnswered}>Next</button>}
        {isLastQuestion && isAnswered && (
            <div className={styles.score}>
                Final Score: {score} / {questions.length}
            </div>
        )}
      </div>
    </div>
  );
};

export default Quiz;
